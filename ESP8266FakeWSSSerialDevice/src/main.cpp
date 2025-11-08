#include <Arduino.h>
#include <array>
#include <vector>
#include <algorithm>

namespace
{
constexpr uint8_t END     = 0xC0;
constexpr uint8_t ESC     = 0xDB;
constexpr uint8_t END_SUB = 0xDC;
constexpr uint8_t ESC_SUB = 0xDD;

constexpr size_t   kMaxAccumulatedBytes          = 512;
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

std::vector<uint8_t> g_preEscaped;
std::array<uint8_t, 3> g_lastPa{};
std::array<uint8_t, 3> g_lastPw{};
bool g_ledState = false;

uint8_t computeChecksum(const uint8_t* buffer, size_t length, size_t trailerBytesToSkip)
{
    const size_t end = length > trailerBytesToSkip ? length - trailerBytesToSkip : 0;
    uint32_t sum = 0;
    for (size_t i = 0; i < end; ++i) sum += buffer[i];
    sum = ((sum & 0x00FFu) + (sum >> 8)) ^ 0xFFu;
    return static_cast<uint8_t>(sum & 0xFFu);
}

uint8_t computeChecksum(const std::vector<uint8_t>& buffer, size_t trailerBytesToSkip)
{
    return computeChecksum(buffer.data(), buffer.size(), trailerBytesToSkip);
}

bool unescapeAndValidate(const std::vector<uint8_t>& preEscaped, std::vector<uint8_t>& frame)
{
    frame.clear();
    frame.reserve(preEscaped.size());

    for (size_t i = 0; i < preEscaped.size(); ++i)
    {
        const uint8_t b = preEscaped[i];
        if (b == ESC && (i + 1) < preEscaped.size())
        {
            const uint8_t next = preEscaped[++i];
            if (next == END_SUB) frame.push_back(END);
            else if (next == ESC_SUB) frame.push_back(ESC);
            else frame.push_back(next);
        }
        else
        {
            frame.push_back(b);
        }
    }

    if (frame.size() < 3) return false;

    const uint8_t expected = frame.back();
    const uint8_t computed = computeChecksum(frame, 1);
    if (computed != expected)
    {
        frame.clear();
        return false;
    }

    return true;
}

std::vector<uint8_t> buildFrame(uint8_t sender, uint8_t target, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> raw;
    raw.reserve(2 + payload.size() + 2);
    raw.push_back(sender);
    raw.push_back(target);
    raw.insert(raw.end(), payload.begin(), payload.end());
    raw.push_back(0x00); // placeholder checksum
    raw.push_back(END);
    raw[raw.size() - 2] = computeChecksum(raw, 2);

    std::vector<uint8_t> escaped;
    escaped.reserve(raw.size() * 2);
    for (size_t i = 0; i < raw.size() - 1; ++i)
    {
        const uint8_t b = raw[i];
        if (b == END)
        {
            escaped.push_back(ESC);
            escaped.push_back(END_SUB);
        }
        else if (b == ESC)
        {
            escaped.push_back(ESC);
            escaped.push_back(ESC_SUB);
        }
        else
        {
            escaped.push_back(b);
        }
    }

    escaped.push_back(END);
    return escaped;
}

bool shouldDropChunk()
{
    if (kReplyDropProbabilityPermille == 0) return false;
    return static_cast<uint16_t>(random(0, 1000)) < kReplyDropProbabilityPermille;
}

void transmitFrame(const std::vector<uint8_t>& bytes)
{
    if (bytes.empty()) return;

    if (kBaseReplyLatencyMs > 0 || kReplyJitterMs > 0)
    {
        uint32_t waitMs = kBaseReplyLatencyMs;
        if (kReplyJitterMs > 0)
            waitMs += static_cast<uint32_t>(random(0, static_cast<long>(kReplyJitterMs) + 1));
        delay(waitMs);
    }

    if (kMaxReplyChunkSize > 0 && bytes.size() > kMaxReplyChunkSize)
    {
        size_t offset = 0;
        const uint32_t chunkDelayWindow = (kReplyJitterMs > 0)
                                              ? std::min<uint32_t>(3, kReplyJitterMs)
                                              : 0;
        while (offset < bytes.size())
        {
            size_t remaining = bytes.size() - offset;
            size_t chunkSize = std::min(kMaxReplyChunkSize, remaining);
            if (chunkSize > 1)
            {
                const long rangeMax = static_cast<long>(chunkSize);
                chunkSize = static_cast<size_t>(random(1, rangeMax + 1));
            }

            if (!shouldDropChunk())
            {
                Serial.write(bytes.data() + offset, chunkSize);
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
            Serial.write(bytes.data(), bytes.size());
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
    for (size_t i = 0; i < g_lastPa.size(); ++i)
    {
        if (g_lastPa[i] > 0 && g_lastPw[i] > 0)
        {
            turnOn = true;
            break;
        }
    }
    applyLedState(turnOn);
}

void copyTriplet(std::array<uint8_t, 3>& dest, const std::vector<uint8_t>& payload, size_t start)
{
    if (payload.size() < start + dest.size()) return;
    for (size_t i = 0; i < dest.size(); ++i)
    {
        dest[i] = payload[start + i];
    }
}

void updateStreamingState(uint8_t messageId, const std::vector<uint8_t>& payload)
{
    if (messageId == static_cast<uint8_t>(WssMessageId::StreamChangeAll) ||
        messageId == static_cast<uint8_t>(WssMessageId::StreamChangeNoIPI))
    {
        if (payload.size() < 6) return;

        copyTriplet(g_lastPa, payload, 0);
        copyTriplet(g_lastPw, payload, 3);
        refreshLedFromStreamingState();
    }
}

void handleFrame(const std::vector<uint8_t>& frame)
{
    if (frame.size() < 5) return; // [sender][target][payload>=1][checksum]

    const uint8_t sender = frame[0];
    const uint8_t target = frame[1];

    std::vector<uint8_t> payload(frame.begin() + 2, frame.end() - 1);
    if (payload.empty()) return;

    const uint8_t messageId = payload[0];
    if (messageId >= static_cast<uint8_t>(WssMessageId::StreamChangeAll) &&
        messageId <= static_cast<uint8_t>(WssMessageId::StreamChangeNoPA))
    {
        updateStreamingState(messageId, payload);
        return; // fire-and-forget range
    }

    switch (messageId)
    {
        case static_cast<uint8_t>(WssMessageId::StimulationSwitch):
        {
            if (payload.size() >= 3)
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
            if (payload.size() >= 3)
            {
                const uint8_t msgId = payload[0];
                const uint8_t eventId = payload[2];
                payload = {msgId, 0x01, eventId};
            }
            break;
        }
        case static_cast<uint8_t>(WssMessageId::ModuleQuery):
        {
            std::array<uint8_t, 16> data{};
            data[0] = 0x01;          // Serial
            data[12] = 50;           // IPD (example 50us)
            data[13] = 0x01;         // Parameter step (PA step = 1)
            const int bit10mA = random(0, 2);
            const int bitPulseGuard = random(0, 2);
            data[10] = static_cast<uint8_t>((bit10mA << 1) | (bitPulseGuard << 2));
            data[14] = static_cast<uint8_t>(bit10mA == 1 ? 10 : 72);
            data[15] = 0xFF;         // PW limit

            std::vector<uint8_t> modulePayload;
            modulePayload.reserve(2 + data.size());
            modulePayload.push_back(static_cast<uint8_t>(WssMessageId::RequestAnalog));
            modulePayload.push_back(static_cast<uint8_t>(data.size()));
            modulePayload.insert(modulePayload.end(), data.begin(), data.end());
            payload.swap(modulePayload);
            break;
        }
        default:
            break;
    }

    const std::vector<uint8_t> reply = buildFrame(target, sender, payload);
    transmitFrame(reply);
}

void processAccumulated()
{
    std::vector<uint8_t> frame;
    if (unescapeAndValidate(g_preEscaped, frame))
    {
        handleFrame(frame);
    }
}
} // namespace

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(0);
    g_preEscaped.reserve(kMaxAccumulatedBytes);
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
            if (!g_preEscaped.empty()) processAccumulated();
            g_preEscaped.clear();
        }
        else
        {
            if (g_preEscaped.size() < kMaxAccumulatedBytes)
            {
                g_preEscaped.push_back(byteIn);
            }
            else
            {
                g_preEscaped.clear(); // drop oversized frame
            }
        }
    }
}
