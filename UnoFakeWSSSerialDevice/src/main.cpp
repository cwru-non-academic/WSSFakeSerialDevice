#include <Arduino.h>

namespace
{
constexpr uint8_t END     = 0xC0;
constexpr uint8_t ESC     = 0xDB;
constexpr uint8_t END_SUB = 0xDC;
constexpr uint8_t ESC_SUB = 0xDD;

constexpr size_t   kMaxAccumulatedBytes          = 256;
constexpr size_t   kMaxPayloadBytes              = kMaxAccumulatedBytes - 3;
constexpr size_t   kMaxRawReplyBytes             = kMaxPayloadBytes + 4;
constexpr size_t   kMaxEscapedReplyBytes         = (kMaxRawReplyBytes * 2) + 1;
constexpr uint32_t kBaseReplyLatencyMs           = 0;  // Set >0 to add fixed reply latency
constexpr uint32_t kReplyJitterMs                = 0;  // Random additional latency [0..value]
constexpr size_t   kMaxReplyChunkSize            = 0;  // If >0, split replies into random chunks up to this size
constexpr uint16_t kReplyDropProbabilityPermille = 0;  // 0-1000 permille chance to drop a chunk
constexpr uint8_t  kLedPin                       = LED_BUILTIN;
constexpr bool     kLedActiveLow                 = true;

enum class WssMessageId : uint8_t
{
    Error = 0x05,
    ModuleQuery = 0x01,
    Reset = 0x04,
    Echo = 0x07,
    RequestAnalog = 0x02,
    Clear = 0x40,
    RequestConfig = 0x41,
    CreateContactConfig = 0x42,
    DeleteContactConfig = 0x43,
    CreateEvent = 0x44,
    DeleteEvent = 0x45,
    AddEventToSchedule = 0x46,
    RemoveEventFromSchedule = 0x47,
    MoveEventToSchedule = 0x48,
    EditEventConfig = 0x49,
    CreateSchedule = 0x4A,
    DeleteSchedule = 0x4B,
    SyncGroup = 0x4C,
    ChangeGroupState = 0x4D,
    ChangeScheduleConfig = 0x4E,
    ResetSchedule = 0x4F,
    CustomWaveform = 0x9D,
    StimulationSwitch = 0x0B,
    BoardCommands = 0x09,
    StreamChangeAll = 0x30,
    StreamChangeNoIPI = 0x31,
    StreamChangeNoPW = 0x32,
    StreamChangeNoPA = 0x33,
};

uint8_t g_preEscaped[kMaxAccumulatedBytes];
size_t g_preEscapedSize = 0;
uint8_t g_lastPa[3]{};
uint8_t g_lastPw[3]{};
uint8_t g_txBuffer[kMaxEscapedReplyBytes];
bool g_ledState = false;

inline size_t clampSize(size_t a, size_t b)
{
    return (a < b) ? a : b;
}

inline uint32_t minU32(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

uint8_t computeChecksum(const uint8_t* buffer, size_t length, size_t trailerBytesToSkip)
{
    const size_t end = length > trailerBytesToSkip ? length - trailerBytesToSkip : 0;
    uint32_t sum = 0;
    for (size_t i = 0; i < end; ++i) sum += buffer[i];
    sum = ((sum & 0x00FFu) + (sum >> 8)) ^ 0xFFu;
    return static_cast<uint8_t>(sum & 0xFFu);
}

bool unescapeAndValidate(const uint8_t* preEscaped, size_t preEscapedSize,
                         uint8_t* frame, size_t& frameSize)
{
    frameSize = 0;
    for (size_t i = 0; i < preEscapedSize; ++i)
    {
        if (frameSize >= kMaxAccumulatedBytes) return false;

        const uint8_t b = preEscaped[i];
        if (b == ESC && (i + 1) < preEscapedSize)
        {
            const uint8_t next = preEscaped[++i];
            if (next == END_SUB) frame[frameSize++] = END;
            else if (next == ESC_SUB) frame[frameSize++] = ESC;
            else frame[frameSize++] = next;
        }
        else
        {
            frame[frameSize++] = b;
        }
    }

    if (frameSize < 3) return false;

    const uint8_t expected = frame[frameSize - 1];
    const uint8_t computed = computeChecksum(frame, frameSize, 1);
    if (computed != expected)
    {
        frameSize = 0;
        return false;
    }

    return true;
}

size_t buildFrame(uint8_t sender, uint8_t target, const uint8_t* payload, size_t payloadSize,
                  uint8_t* outBuffer, size_t maxOut)
{
    if (payloadSize > kMaxPayloadBytes || maxOut < 3) return 0;

    uint8_t raw[kMaxRawReplyBytes];
    size_t rawSize = 0;

    raw[rawSize++] = sender;
    raw[rawSize++] = target;

    for (size_t i = 0; i < payloadSize && rawSize < kMaxRawReplyBytes - 2; ++i)
    {
        raw[rawSize++] = payload[i];
    }

    if (rawSize + 2 > kMaxRawReplyBytes) return 0;

    raw[rawSize++] = 0x00; // placeholder checksum
    raw[rawSize++] = END;
    raw[rawSize - 2] = computeChecksum(raw, rawSize, 2);

    size_t outSize = 0;
    for (size_t i = 0; i < rawSize - 1; ++i)
    {
        if (outSize + 2 >= maxOut) return 0;

        const uint8_t b = raw[i];
        if (b == END)
        {
            outBuffer[outSize++] = ESC;
            outBuffer[outSize++] = END_SUB;
        }
        else if (b == ESC)
        {
            outBuffer[outSize++] = ESC;
            outBuffer[outSize++] = ESC_SUB;
        }
        else
        {
            outBuffer[outSize++] = b;
        }
    }

    if (outSize >= maxOut) return 0;
    outBuffer[outSize++] = END;
    return outSize;
}

bool shouldDropChunk()
{
    if (kReplyDropProbabilityPermille == 0) return false;
    return static_cast<uint16_t>(random(0, 1000)) < kReplyDropProbabilityPermille;
}

void transmitFrame(const uint8_t* bytes, size_t length)
{
    if (length == 0) return;

    if (kBaseReplyLatencyMs > 0 || kReplyJitterMs > 0)
    {
        uint32_t waitMs = kBaseReplyLatencyMs;
        if (kReplyJitterMs > 0)
            waitMs += static_cast<uint32_t>(random(0, static_cast<long>(kReplyJitterMs) + 1));
        delay(waitMs);
    }

    if (kMaxReplyChunkSize > 0 && length > kMaxReplyChunkSize)
    {
        size_t offset = 0;
        const uint32_t chunkDelayWindow = (kReplyJitterMs > 0)
                                              ? minU32(3u, kReplyJitterMs)
                                              : 0;
        while (offset < length)
        {
            size_t remaining = length - offset;
            size_t chunkSize = clampSize(kMaxReplyChunkSize, remaining);
            if (chunkSize > 1)
            {
                const long rangeMax = static_cast<long>(chunkSize);
                chunkSize = static_cast<size_t>(random(1, rangeMax + 1));
            }

            if (!shouldDropChunk())
            {
                Serial.write(bytes + offset, chunkSize);
                Serial.flush();
            }

            offset += chunkSize;

            if (chunkDelayWindow > 0)
            {
                delay(static_cast<uint32_t>(random(0, static_cast<long>(chunkDelayWindow) + 1)));
            }
        }
    }
    else
    {
        if (!shouldDropChunk())
        {
            Serial.write(bytes, length);
            Serial.flush();
        }
    }
}

void applyLedState(bool on)
{
    if (g_ledState == on) return;
    g_ledState = on;
    const uint8_t level = (kLedActiveLow ? (on ? LOW : HIGH) : (on ? HIGH : LOW));
    digitalWrite(kLedPin, level);
}

void refreshLedFromStreamingState()
{
    bool turnOn = false;
    for (size_t i = 0; i < 3; ++i)
    {
        if (g_lastPa[i] > 0 && g_lastPw[i] > 0)
        {
            turnOn = true;
            break;
        }
    }
    applyLedState(turnOn);
}

void copyTriplet(uint8_t* dest, const uint8_t* payload, size_t payloadSize, size_t start)
{
    if (payloadSize < start + 3) return;
    for (size_t i = 0; i < 3; ++i)
    {
        dest[i] = payload[start + i];
    }
}

void updateStreamingState(uint8_t messageId, const uint8_t* payload, size_t payloadSize)
{
    if (messageId == static_cast<uint8_t>(WssMessageId::StreamChangeAll) ||
        messageId == static_cast<uint8_t>(WssMessageId::StreamChangeNoIPI))
    {
        if (payloadSize < 6) return;

        copyTriplet(g_lastPa, payload, payloadSize, 0);
        copyTriplet(g_lastPw, payload, payloadSize, 3);
        refreshLedFromStreamingState();
    }
}

void handleFrame(const uint8_t* frame, size_t frameSize)
{
    if (frameSize < 5) return; // [sender][target][payload>=1][checksum]

    const uint8_t sender = frame[0];
    const uint8_t target = frame[1];

    size_t payloadSize = frameSize - 3;
    if (payloadSize == 0 || payloadSize > kMaxPayloadBytes) return;

    uint8_t payload[kMaxPayloadBytes];
    memcpy(payload, frame + 2, payloadSize);

    const uint8_t messageId = payload[0];
    if (messageId >= static_cast<uint8_t>(WssMessageId::StreamChangeAll) &&
        messageId <= static_cast<uint8_t>(WssMessageId::StreamChangeNoPA))
    {
        updateStreamingState(messageId, payload, payloadSize);
        return; // fire-and-forget range
    }

    switch (messageId)
    {
        case static_cast<uint8_t>(WssMessageId::StimulationSwitch):
        {
            if (payloadSize >= 3)
            {
                if (payload[2] == 0x03)      payload[2] = 0x01;
                else if (payload[2] == 0x04) payload[2] = 0x00;
            }
            break;
        }
        case static_cast<uint8_t>(WssMessageId::CreateContactConfig):
        case static_cast<uint8_t>(WssMessageId::CreateEvent):
        case static_cast<uint8_t>(WssMessageId::CreateSchedule):
        case static_cast<uint8_t>(WssMessageId::EditEventConfig):
        case static_cast<uint8_t>(WssMessageId::AddEventToSchedule):
        {
            if (payloadSize >= 3)
            {
                const uint8_t msgId = payload[0];
                const uint8_t eventId = payload[2];
                payload[0] = msgId;
                payload[1] = 0x01;
                payload[2] = eventId;
                payloadSize = 3;
            }
            break;
        }
        case static_cast<uint8_t>(WssMessageId::ModuleQuery):
        {
            uint8_t data[16]{};
            data[0] = 0x01;          // Serial
            data[12] = 50;           // IPD (example 50us)
            data[13] = 0x01;         // Parameter step (PA step = 1)
            const int bit10mA = random(0, 2);
            const int bitPulseGuard = random(0, 2);
            data[10] = static_cast<uint8_t>((bit10mA << 1) | (bitPulseGuard << 2));
            data[14] = static_cast<uint8_t>(bit10mA == 1 ? 10 : 72);
            data[15] = 0xFF;         // PW limit

            payload[0] = static_cast<uint8_t>(WssMessageId::RequestAnalog);
            payload[1] = static_cast<uint8_t>(sizeof(data));
            memcpy(&payload[2], data, sizeof(data));
            payloadSize = 2 + sizeof(data);
            break;
        }
        default:
            break;
    }

    const size_t replySize = buildFrame(target, sender, payload, payloadSize, g_txBuffer, sizeof(g_txBuffer));
    transmitFrame(g_txBuffer, replySize);
}

void processAccumulated()
{
    uint8_t frame[kMaxAccumulatedBytes];
    size_t frameSize = 0;
    if (unescapeAndValidate(g_preEscaped, g_preEscapedSize, frame, frameSize))
    {
        handleFrame(frame, frameSize);
    }
}
} // namespace

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(0);
    randomSeed(micros());
    pinMode(kLedPin, OUTPUT);
    applyLedState(false);
}

void loop()
{
    while (Serial.available() > 0)
    {
        const uint8_t byteIn = static_cast<uint8_t>(Serial.read());
        if (byteIn == END)
        {
            if (g_preEscapedSize > 0) processAccumulated();
            g_preEscapedSize = 0;
        }
        else
        {
            if (g_preEscapedSize < kMaxAccumulatedBytes)
            {
                g_preEscaped[g_preEscapedSize++] = byteIn;
            }
            else
            {
                g_preEscapedSize = 0; // drop oversized frame
            }
        }
    }
}
