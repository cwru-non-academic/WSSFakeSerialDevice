# WSSFakeSerialDevice

Firmware for Arduino UNO and ESP8266 boards that mimics WSS test-transport replies over a real serial connection. Use it to exercise host serial communication without a WSS.

The emulator checks frame checksums and handles messages with canned or echoed replies, but does not enforce message order or validate application-level parameter boundaries.

## Supported boards

| Project | PlatformIO environment | Status |
| --- | --- | --- |
| [Arduino UNO](UnoFakeWSSSerialDevice) | `uno` | Previously tested on hardware |
| [Adafruit HUZZAH ESP8266](ESP8266FakeWSSSerialDevice) | `huzzah` | Implemented; hardware testing not confirmed |

UNO boards reset when the serial port opens. A host that sends immediately can lose the first message; configure more than one setup attempt or allow the board to finish resetting. The Python helper below waits two seconds after opening the port.

## Build and upload

Install [PlatformIO](https://docs.platformio.org/en/latest/core/installation/index.html), connect the chosen board, and run these commands from the repository root.

For Arduino UNO:

```sh
pio run -d UnoFakeWSSSerialDevice -e uno
pio run -d UnoFakeWSSSerialDevice -e uno -t upload
```

For Adafruit HUZZAH ESP8266:

```sh
pio run -d ESP8266FakeWSSSerialDevice -e huzzah
pio run -d ESP8266FakeWSSSerialDevice -e huzzah -t upload
```

If automatic port detection selects the wrong board, append `--upload-port PORT` to the upload command, replacing `PORT` with the board's serial port, such as `COM3` or `/dev/ttyUSB0`.

Both firmware implementations communicate at **115200 baud**. Close serial monitors before running a host application or the Python helper.

## Check an UNO from Python

The host helper requires Python 3.10 or newer and [pySerial](https://pyserial.readthedocs.io/en/latest/). From the repository root:

```sh
python -m pip install pyserial
python UnoFakeWSSSerialDevice/test/test_wss_serial.py --port /dev/ttyACM0 --clear-iterations 5
```

Replace `/dev/ttyACM0` with the connected board's port (`COM3`, for example, on Windows).

The helper sends CLEAR commands, validates the reply addresses and payload, and prints round-trip latency. It then sends ten streaming messages at half-second intervals, alternating the onboard LED on and off. It exits with status 0 if all CLEAR checks pass and status 1 if any fail; LED behavior is a visual check.

| Option | Default | Purpose |
| --- | --- | --- |
| `--port` | Required | Connected UNO serial port |
| `--baud` | `115200` | Serial baud rate; must match the firmware |
| `--clear-only` | Off | Skip streaming messages |
| `--clear-iterations` | `1` | Number of CLEAR round trips |
| `--stream-iterations` | `10` | Number of LED toggle messages |

The UNO helper is a host-side hardware check, not a PlatformIO unit-test suite. There are no automated firmware unit tests in this repository.

## LED behavior and reply simulation

Streaming messages control the onboard LED. It turns on when at least one channel has both a positive amplitude and a positive pulse width, and turns off when no channel satisfies that condition. The firmware does not blink autonomously; the helper produces the half-second pattern by alternating messages.

Each project's `src/main.cpp` contains optional reply-simulation settings:

| Constant | Default | Effect |
| --- | --- | --- |
| `kBaseReplyLatencyMs` | `0` | Fixed delay before a reply, in milliseconds |
| `kReplyJitterMs` | `0` | Random additional delay, in milliseconds |
| `kMaxReplyChunkSize` | `0` | Enable randomized reply chunks when positive |
| `kReplyDropProbabilityPermille` | `0` | Probability of dropping a reply chunk, from 0 to 1000 |

Edit the constants in the selected project's source and rebuild/upload to change these settings.

## Project layout and development

Each board project has a `platformio.ini` defining its board and Arduino framework, and a `src/main.cpp` containing the firmware. The UNO project's `test/test_wss_serial.py` is the host helper described above.

PlatformIO supports optional `include/` directories for project headers and `lib/` directories for private libraries. Neither project currently contains additional headers or private libraries; add actual source files there when needed. See the [library dependency finder](https://docs.platformio.org/en/latest/librarymanager/ldf.html) and [PlatformIO test runner](https://docs.platformio.org/en/latest/advanced/unit-testing/index.html) references for those facilities.

For VS Code, open the chosen board project with the PlatformIO extension. The shared extension recommendations remain in each project's `.vscode/extensions.json`. PlatformIO generates machine-specific IntelliSense and launch configurations locally; those files, build output, and Python bytecode caches are excluded from version control.

## License

[MIT](LICENSE).
