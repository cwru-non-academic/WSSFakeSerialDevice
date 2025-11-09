# WSSFakeSerialDevice
## Summary
A project built for multiple microcontrollers, including Arduino UNO. These scripts mimic the functionality of the test transport to verify serial communication without a WSS. The functionality replicates the test transport so it can handle various messages and send the correct replies, but it will not check message order or validate that the data inside the message is within expected boundaries.

## Supported Microcontrollers

### Arduino UNO
Tested. It will always miss the first message, causing the API to automatically retry if the maximum setup attempts > 1. The first message fails because UNO boards reset when the port opens, and the API does not wait before sending the first message. The timeout and the board reset both take about 2 seconds, so the second retry succeeds.

### ESP8266
Not tested, but implemented.
