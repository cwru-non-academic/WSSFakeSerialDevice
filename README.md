# WSSFakeSerialDevice
## Summary
A project built for multiple microcontrollers, including Arduino UNO. These scripts mimic the functionality of the test transport to verify serial communication without a WSS. The functionality replicates the test transport so it can handle various messages and send the correct replies, but it will not check message order or validate that the data inside the message is within expected boundaries. The Arduino UNO folder includes a Python-based test script that verifies two-way message functionality by printing information based on the expected structure of the reply. One-way messages can be tested by observing the onboard LED, which blinks with a half-second on/off pattern. The LED is controlled through one-way messages: values of 0 across all channels turn the LED off, while any value greater than 0 for both amplitude and pulse width on at least one channel turns it on.

## Build, upload, and verify

Install PlatformIO Core and connect the Arduino Uno by USB. From the repo root, use `pio device list` to identify its serial port, then build and upload:

```bash
cd UnoFakeWSSSerialDevice
pio run -e uno
pio run -e uno -t upload
```

If several ports are present, add `--upload-port` followed by the Uno's port to the upload command. Close any serial monitor before uploading or testing.

The Uno host tester requires Python 3.10+ and PySerial. In a Python environment with PySerial installed (`python -m pip install pyserial`), run this from the Uno project directory, replacing `/dev/ttyACM0` with the Uno's port, such as `COM5` on Windows:

```bash
python test/test_wss_serial.py --port /dev/ttyACM0 --clear-only
```

Look for `CLEAR result: PASS`. Omit `--clear-only` to exercise the built-in LED with the same port. This board verifies serial exchange and LED behavior. It produces no stimulation output.

The ESP8266 project builds with `pio run -e huzzah` from `ESP8266FakeWSSSerialDevice/`. Physical ESP8266 verification is pending.

## Supported Microcontrollers

### Arduino UNO
Tested. It will always miss the first message, causing the API to automatically retry if the maximum setup attempts > 1. The first message fails because UNO boards reset when the port opens, and the API does not wait before sending the first message. The timeout and the board reset both take about 2 seconds, so the second retry succeeds. 

### ESP8266
Not tested, but implemented.
