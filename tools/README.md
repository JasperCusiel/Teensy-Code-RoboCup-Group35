# Map telemetry viewer

The firmware emits `MAP` lines at 6 Hz and repeats a `CONFIG` line once per
second when
`MAP_TELEMETRY_ENABLED` in `include/telemetry.h` is `1`.

Install the desktop dependencies:

```sh
python3 -m pip install -r tools/requirements.txt
```

Start the viewer after connecting the Teensy:

```sh
python3 tools/map_viewer.py /dev/cu.usbmodem1234
```

On Windows, use a port such as `COM3`. The left panel reconstructs the same
log-odds map as firmware. The right panel shows the latest ToF rays and endpoints.
Press `r` with the viewer focused to clear its local map.
