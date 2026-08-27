# MiniHawk — Agent Context

## Build & Run
- **Runtime**: `python main.py` (or `python -m flet run main.py` for hot reload)
- **Virtual env**: `source venv/bin/activate`
- **Python**: 3.12+
- **Dependencies**: `pip install -r requirements.txt`

## Key Patterns
- **Flet 0.83.0** — use `ft.run(main)`, `ft.Padding.all()`, `ft.Border.all()`, `ft.BorderRadius.all()`, `ft.FilledButton`, NOT deprecated aliases.
- **Charts**: Use `flet_charts` (`fch.LineChart`, etc.); recreate series objects on data push rather than mutating points in-place.
- **MAVLink listener**: Runs in background `asyncio` task; `restart_event` used to rebind socket on settings change.
- **Theme**: `theme.json` + `config.json` persist user preferences.
- **PNG widgets**: Heavy Matplotlib figures are rendered to base64 PNG via `BytesIO`, then displayed with `ft.Image(src=base64_png)`.

## 3D Terrain Scatter Design
- **Library**: `srtm.py` (NASA SRTM 90m, no API key) + `scipy.griddata` (interpolation).
- **Cache**: `elevation_cache/` directory holds `.hgt` tiles forever; downloaded on first use.
- **Rendering**: Matplotlib 3D `plot_surface` for terrain mesh, `scatter3D` for CO₂ points.
- **Point placement**: TRUE GPS ALTITUDE — points float above terrain at their actual barometric height.
- **No 2D scatter**: Entirely removed. No `build_scatter_stack`, no 2D/3D toggle.
- **Color**: CO₂ points always use `coolwarm` colormap + colorbar; bands only affect line charts (CO₂/HUM/TEMP/H2S/SO2/CO2_FINE), not scatter.
- **Fallback**: If offline with empty cache, render flat plane with "Terrain unavailable" watermark.
- **Performance target**: <500ms per render at 80×80 mesh resolution.
- **Offline terrain download**: `_get_tile_names()` computes SRTM tile filenames for a lat/lon bounding box. `_list_cached_tiles()` scans `elevation_cache/` for existing `.hgt` files. Downloads run via `run_in_executor` (non-blocking). `_terrain_only_render()` renders a terrain-only 3D surface PNG (no scatter) for download preview.

## Sensors & Layout
- **6 line charts** (left column, scrollable): CO₂ (purple), HUMIDITY (green), TEMPERATURE (orange), H2S (yellow), SO₂ (red), CO₂_FINE (cyan).
- **Scatter column** (right): 3D terrain + CO₂ GPS points.
- **Firmware** (`firmware/main.ino`): SEN66 via I²C for TEMP/HUMIDITY/CO₂; 4-20mA analog sensors on pins 26/27/14 for H₂S (100 ppm FS), SO₂ (50 ppm FS), CO₂_FINE (3000 ppm FS). All 6 sensors sent as `NAMED_VALUE_FLOAT` over MAVLink.
- **Fake sender** (`tools/fake_mavlink_sender.py`): Synthetic telemetry for all 6 sensors + GPS + attitude. Run with `--noise` for realistic jitter.

## Project Structure
```
minihawk/
├── main.py                    # Entry point — Flet GUI + MAVLink listener
├── requirements.txt           # Python dependencies (flet, pymavlink, srtm.py, scipy, matplotlib)
├── pyproject.toml             # Flet packaging configuration
├── theme.json                 # Saved theme preference (auto-created)
├── config.json                # Saved MAVLink + band config (auto-created)
├── elevation_cache/           # SRTM .hgt terrain tiles (auto-populated)
├── assets/
│   ├── logo.png               # App branding
│   └── icon.png               # Build icon
├── tools/
│   ├── build-script.sh        # Linux build helper with header patch
│   ├── requirements.txt       # Dev / tool dependencies
│   └── fake_mavlink_sender.py # Test utility — synthetic 6-sensor telemetry
└── firmware/
    └── main.ino               # Reference ESP32 firmware (SEN66 + 4-20mA)
```
