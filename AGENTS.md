# Agent notes

## Cursor Cloud specific instructions

This repo is **ESP32-S3 firmware** for the Waveshare ESP32-S3-Touch-LCD-1.54 (`fluid_box_3d`): on-device launcher + Fluid Box + Task Maze. There is no web/backend stack, Docker Compose, or Node package manager.

### Toolchain activation

- ESP-IDF **6.0.2** is installed via Espressif Installation Manager (`eim`). Prefer:
  - `eim run 'idf.py <args>' v6.0.2`
  - or `source ~/.espressif/tools/activate_idf_v6.0.2.sh` (do **not** use `export.sh` alone — EIM’s Python venv path differs from the legacy `python_env/` layout).
- Target is pinned to `esp32s3` in `sdkconfig.defaults`. Fresh trees: `eim run 'idf.py set-target esp32s3' v6.0.2` then `eim run 'idf.py build' v6.0.2`.
- Host tools need **pyserial** (`pip3 install --user pyserial`). Inside `eim run`, the IDF venv may already provide it; system/`--user` install covers plain `python3 tools/...`.

### Lint / test / run

- No ESLint/clang-tidy/pytest suite in-repo. Compile-time `static_assert`s are exercised by **building**.
- Hardware acceptance (needs USB board): `python3 tools/firmware_shell_acceptance.py` (see that file’s CLI). Interactive helpers: `python3 tools/device_dev.py --help`.
- Flash/monitor: `eim run 'idf.py -p <PORT> flash monitor' v6.0.2`.

### Device access & visual testing

- Cloud VMs typically have **no USB Serial/JTAG** device (`/dev/ttyACM*` / `ttyUSB*`). Without a board, `device_dev.py` / acceptance fail at port discovery.
- `idf.py qemu` can generate a flash image and start `qemu-system-xtensa`, but this firmware’s ST7789 / QMI8658 / CST816 / USB console path is **not** meaningfully emulated — expect ROM boot then stall/hang. Do not treat QEMU as an LCD UI substitute.
- For visual smoke without hardware, a **host panel stand-in** lives outside the repo at `~/waveshare-panel-host/panel_emulator.py` (pygame, `DISPLAY=:1`). It mirrors launcher/Fluid/Maze look-and-feel for recording/Desktop demos only — it is **not** cycle-accurate firmware. Real `@FB` screenshots and acceptance still require the Waveshare board.

### Gotchas

- `build/`, `managed_components/`, and `sdkconfig` are gitignored; first build downloads `waveshare/qmi8658` via the component manager.
- Keep DTR/RTS alone when opening serial (`device_dev.py` already does this) so the USB Serial/JTAG link does not spuriously reset the chip.
