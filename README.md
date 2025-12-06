# 3rdeye-mcu

Overview
-
`3rdeye-mcu` is firmware for an MCU (ESP32/PlatformIO) to interface with a Texas Instruments AWR16xx mmWave sensor. The project reads raw UART frames from the radar, finds and validates mmWave SDK packet headers, parses detected-object TLVs, and provides a pathway to stream parsed frames out (USB/Serial) or to perform processing on the MCU.

Key features
-
- Parse mmWave SDK TLV frames and extract detected object lists.
- Two start modes: `radar_begin` (full pipeline) and `directStreamingRadarBegin` (direct streaming).
- Task-based design (FreeRTOS): separate tasks for receiving, parsing, and streaming data.
- Frame pool and queueing to avoid frequent large stack allocations and to keep streaming/parsing decoupled.
- Helpers for converting byte arrays to integers and for parsing TLV/object payloads.

Repository layout (high level)
-
- `platformio.ini` – PlatformIO build configuration for this project.
- `src/main.cpp` – Main firmware: UART setup, frame handling, parsing, tasks, and CLI helpers.
- `include/` – Header files (pin definitions, `configs.h`, config strings, structs and constants).
- `lib/` – Libraries (project-local helpers such as `radarSerial`).
- `data/` – Example/default radar config files.

Quick start (development)
-
1. Install PlatformIO and open this project in VS Code.
2. Review hardware pin definitions and defaults in `include/common.h` and `include/configs.h` and update them to match your board/wiring (pins such as `SOP_0`, `SOP_1`, `SOP_2`, `NRST`, `CLI_RX`, `CLI_TX`, `dataSerial_RX` are referenced in `src/main.cpp`).
3. Build and upload with PlatformIO (replace environment if needed):
```bash
pio run --target upload
```
4. Open the serial monitor to see debug output (default debug serial is `115200`):
```bash
pio device monitor -b 115200
```

How it runs
-
- On startup `setup()` currently calls `directStreamingRadarBegin(...)`. That initializes the CLI and data UARTs, spawns tasks, resets the radar, and sends the radar configuration.
- The code implements three primary runtime tasks:
	- a data-receive task (`mssHandler` / `mssHandlerDirectStreaming`) that collects UART bytes, detects the mmWave magic word, and copies complete frames into a free frame from the frame pool;
	- a parsing task (`parseData`) that takes raw frames, runs `parseDataFrame()` and `parseTLV()` to populate `ProcessedFrame` structures;
	- a streaming task (`streamData`) that sends parsed frames off-board or prints them for debugging.
- Radar configuration lines are stored in `include/configs.h` and sent to the radar line-by-line using the internal CLI helper `sendCommand()`.

Testing and simulation
-
- The code includes a commented-out `simulateDataStream()` helper that can feed example binary test data into the parser without actual UART hardware—useful when developing the parsing logic.

Configuration and tuning
-
- Edit the radar CLI profile and frame configuration strings in `include/configs.h` (the project contains `awr1642_default_config.cfg` as an example) and call `sendConfig()` or allow `setup()` to send `default_config`.
- Adjust `FRAME_POOL_SIZE`, `FRAME_BUFFER_SIZE`, and queue sizes in the relevant headers if you observe buffer exhaustion or memory pressure.

Best practices & developer notes
-
- Pin mapping: keep the pin definitions centralized in `include/common.h` and update wiring to match your board.
- Keep UI/CLI and data UARTs logically separated; `cliSerial` is used for issuing commands (115200) while `dataSerial` is configured for high-speed radar data (921600).
- Monitor heap and stack usage. The code uses `new` to duplicate processed frames and FreeRTOS stack watermarks (`uxTaskGetStackHighWaterMark`) are printed in several places—watch these during long runs.
- Avoid blocking operations in real-time tasks; the project uses queues and short delays to maintain responsiveness.
- When changing configuration strings, ensure `sendConfig()` succeeds (it waits for CLI responses like `Done` or `Skipped`).
- Use the `simulateDataStream()` path when unit-testing parsing logic without hardware.

Troubleshooting
-
- If you see no data: verify wiring and that the radar power-good pin or reset behavior matches expectations; `radarReset()` and `checkRadarState()` are available to help.
- If parsed frame counts are incorrect or parser crashes: check magic word detection (`findMagicWord`), that `totalPacketLen` matches the captured length, and that `FRAME_BUFFER_SIZE` is large enough for incoming packets.
- If free-frame queueing blocks: increase `FRAME_POOL_SIZE` or reduce ingestion rate during testing.

Next steps / suggestions
-
- Add a small host-side script (Python) to open the expected serial endpoint and verify parsed output format.
- Add unit tests for `parseDataFrame()`, `parseTLV()`, and helper converters using sample frame blobs (use `simulateDataStream()` data as fixtures).
- Document exact wiring and the required PlatformIO environment(s) used to build/upload in this README once your hardware wiring is finalized.

License
-
This repository currently contains no explicit license file. Add a `LICENSE` if you intend to share or open-source the code.

Contact / maintainer
-
Refer to the repository owner for questions or to collaborate on the parser/streamer implementation.