import flet as ft
from flet import padding
import flet_charts as fch
from pymavlink import mavutil
import csv
import math

import asyncio
import json
import os
import sys
import subprocess
import time
from datetime import datetime
from pathlib import Path
from io import BytesIO
from collections import deque

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

MAX_POINTS = 50
THEME_FILE = Path(__file__).with_name("theme.json")
CONFIG_FILE = Path(__file__).with_name("config.json")
AUTOSAVE_FILE = Path(__file__).with_name("telemetry_latest.csv")
AUTOSAVE_INTERVAL = 5.0  # seconds between auto-saves

DEFAULT_CONFIG = {
    "mavlink_bind_host": "0.0.0.0",
    "mavlink_port": "14550",
    "use_bands": {
        "CO2": False,
        "HUMIDITY": False,
        "TEMPERATURE": False,
        "H2S": False,
        "SO2": False,
        "CO2_FINE": False,
        "SCATTER": False,
    },
    "bands": {
        "CO2": [
            {"max": 600, "color": "green", "label": "Low"},
            {"max": 1000, "color": "yellow", "label": "Moderate"},
            {"max": 999999, "color": "red", "label": "High"},
        ],
        "HUMIDITY": [
            {"max": 30, "color": "blue", "label": "Dry"},
            {"max": 60, "color": "green", "label": "Comfortable"},
            {"max": 80, "color": "yellow", "label": "Humid"},
            {"max": 999999, "color": "red", "label": "Very Humid"},
        ],
        "TEMPERATURE": [
            {"max": 15, "color": "blue", "label": "Cold"},
            {"max": 25, "color": "green", "label": "Comfortable"},
            {"max": 30, "color": "yellow", "label": "Warm"},
            {"max": 999999, "color": "red", "label": "Hot"},
        ],
        "H2S": [
            {"max": 10, "color": "green", "label": "Safe"},
            {"max": 50, "color": "yellow", "label": "Caution"},
            {"max": 999999, "color": "red", "label": "Danger"},
        ],
        "SO2": [
            {"max": 2, "color": "green", "label": "Safe"},
            {"max": 10, "color": "yellow", "label": "Caution"},
            {"max": 999999, "color": "red", "label": "Danger"},
        ],
        "CO2_FINE": [
            {"max": 600, "color": "green", "label": "Low"},
            {"max": 1000, "color": "yellow", "label": "Moderate"},
            {"max": 999999, "color": "red", "label": "High"},
        ],
    },
}

COLOR_MAP = {
    "green": "#00e676",
    "yellow": "#ffea00",
    "red": "#ff1744",
    "blue": "#2979ff",
    "orange": "#ff9100",
    "purple": "#d500f9",
    "cyan": "#00e5ff",
    "white": "#ffffff",
    "black": "#000000",
}


def load_config():
    if CONFIG_FILE.exists():
        try:
            data = json.loads(CONFIG_FILE.read_text())
            cfg = DEFAULT_CONFIG.copy()
            cfg.update(data)
            return cfg
        except Exception:
            pass
    return DEFAULT_CONFIG.copy()


def save_config(cfg):
    try:
        CONFIG_FILE.write_text(json.dumps(cfg, indent=2))
    except Exception:
        pass


def detect_system_theme():
    """Detect OS theme preference: returns 'dark' or 'light'"""
    try:
        if sys.platform == "win32":
            import winreg
            key = winreg.OpenKey(winreg.HKEY_CURRENT_USER, r"Software\Microsoft\Windows\CurrentVersion\Themes\Personalize")
            value, _ = winreg.QueryValueEx(key, "AppsUseLightTheme")
            return "light" if value == 1 else "dark"
        elif sys.platform == "darwin":
            result = subprocess.run(
                ["defaults", "read", "-g", "AppleInterfaceStyle"],
                capture_output=True, text=True, timeout=5
            )
            return "dark" if "dark" in result.stdout.lower() else "light"
        else:
            # Linux: try gsettings first
            try:
                result = subprocess.run(
                    ["gsettings", "get", "org.gnome.desktop.interface", "gtk-theme"],
                    capture_output=True, text=True, timeout=5
                )
                theme = result.stdout.strip().strip("'")
                return "dark" if "dark" in theme.lower() else "light"
            except Exception:
                # Fallback to environment variables
                env_theme = os.environ.get("GTK_THEME", os.environ.get("QT_STYLE_OVERRIDE", ""))
                if env_theme:
                    return "dark" if "dark" in env_theme.lower() else "light"
    except Exception:
        pass
    return "dark"


def load_theme():
    """Load saved theme or detect OS preference"""
    if THEME_FILE.exists():
        try:
            data = json.loads(THEME_FILE.read_text())
            theme = data.get("theme")
            if theme in ("dark", "light"):
                return theme
        except Exception:
            pass
    return detect_system_theme()


def save_theme(theme: str):
    try:
        THEME_FILE.write_text(json.dumps({"theme": theme}))
    except Exception:
        pass


# GPS quality thresholds (easily tweakable)
GPS_QUALITY_THRESHOLDS = {
    "good": {"fix_type": 3, "min_sats": 10, "max_eph_cm": 100},
    "fair": {"fix_type": 3, "min_sats": 6, "max_eph_cm": 200},
    "poor": {"fix_type": 2, "min_sats": 4, "max_eph_cm": 500},
}


def get_gps_quality_color(fix_type, sats, eph_cm, theme):
    """Return theme color for GPS quality based on thresholds."""
    if fix_type >= GPS_QUALITY_THRESHOLDS["good"]["fix_type"] and sats >= GPS_QUALITY_THRESHOLDS["good"]["min_sats"] and eph_cm <= GPS_QUALITY_THRESHOLDS["good"]["max_eph_cm"]:
        return theme["gps_quality_good"]
    if fix_type >= GPS_QUALITY_THRESHOLDS["fair"]["fix_type"] and sats >= GPS_QUALITY_THRESHOLDS["fair"]["min_sats"] and eph_cm <= GPS_QUALITY_THRESHOLDS["fair"]["max_eph_cm"]:
        return theme["gps_quality_fair"]
    if fix_type >= GPS_QUALITY_THRESHOLDS["poor"]["fix_type"] and sats >= GPS_QUALITY_THRESHOLDS["poor"]["min_sats"] and eph_cm <= GPS_QUALITY_THRESHOLDS["poor"]["max_eph_cm"]:
        return theme["gps_quality_poor"]
    return theme["gps_quality_bad"]


def format_gps_badge(fix_type, sats, eph_cm):
    if fix_type == 0:
        fix_label = "NO GPS"
    elif fix_type == 1:
        fix_label = "NO FIX"
    elif fix_type == 2:
        fix_label = "2D"
    else:
        fix_label = "3D"
    return f"{fix_label} | {sats} sats | {eph_cm}cm"


# Theme color palettes
THEMES = {
    "dark": {
        "page_bg": "#111214",
        "card_bg": "#1c1e21",
        "grid_color": "#2a2d31",
        "axis_color": "#555555",
        "label_color": "#888888",
        "text_primary": ft.Colors.WHITE,
        "text_secondary": ft.Colors.GREY_400,
        "text_blue": ft.Colors.BLUE_200,
        "text_green": ft.Colors.GREEN_300,
        "text_cyan": ft.Colors.CYAN_ACCENT,
        "log_bg": "black",
        "georef_border": "#2a2d31",
        "scatter_no_data": "#888888",
        "gps_quality_good": ft.Colors.GREEN_700,
        "gps_quality_fair": ft.Colors.YELLOW_700,
        "gps_quality_poor": ft.Colors.ORANGE_700,
        "gps_quality_bad": ft.Colors.RED_700,
    },
    "light": {
        "page_bg": "#f5f6f8",
        "card_bg": "#ffffff",
        "grid_color": "#e0e0e0",
        "axis_color": "#999999",
        "label_color": "#555555",
        "text_primary": "#1a1a1a",
        "text_secondary": "#555555",
        "text_blue": ft.Colors.BLUE_700,
        "text_green": ft.Colors.GREEN_700,
        "text_cyan": ft.Colors.CYAN_700,
        "log_bg": "#eef0f4",
        "georef_border": "#e0e0e0",
        "scatter_no_data": "#555555",
        "gps_quality_good": ft.Colors.GREEN_600,
        "gps_quality_fair": ft.Colors.YELLOW_600,
        "gps_quality_poor": ft.Colors.ORANGE_600,
        "gps_quality_bad": ft.Colors.RED_600,
    },
}

ELEVATION_CACHE_DIR = Path(__file__).parent / "elevation_cache"

class TerrainCache:
    """Fetch SRTM terrain tiles for GPS bounding boxes and cache them forever."""
    def __init__(self, cache_dir: Path = None):
        self.cache_dir = cache_dir or ELEVATION_CACHE_DIR
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        import srtm
        self.elev = srtm.get_data(local_cache_dir=str(self.cache_dir))
        self._last_grid_key = None
        self._last_grid_data = (None, None, None)

    def get_elevations(self, lats: list, lons: list) -> list:
        """Bulk elevation query. Returns list with None for missing data."""
        return [self.elev.get_elevation(lat, lon) for lat, lon in zip(lats, lons)]

    def get_elevation_grid(self, lat_min, lat_max, lon_min, lon_max, resolution=50):
        """Return (X, Y, Z) meshgrid arrays for Matplotlib plot_surface."""
        # Quantize bounding box to ~100m to reuse cached mesh during slow movement or stationary flights
        grid_key = (round(lat_min, 3), round(lat_max, 3), round(lon_min, 3), round(lon_max, 3), resolution)
        if grid_key == self._last_grid_key and self._last_grid_data[0] is not None:
            return self._last_grid_data

        import builtins
        _orig_print = builtins.print
        def _quiet_print(*args, **kwargs):
            if args and str(args[0]).strip().startswith("4 "):
                return
            _orig_print(*args, **kwargs)
        builtins.print = _quiet_print
        try:
            import numpy as np
            from scipy.interpolate import griddata

            sample_res = resolution * 2
            lat_s = np.linspace(lat_min, lat_max, sample_res)
            lon_s = np.linspace(lon_min, lon_max, sample_res)

            points = np.array([(lon, lat) for lat in lat_s for lon in lon_s])
            elevs = np.array(self.get_elevations([lat for _, lat in points], [lon for lon, _ in points]))

            mask = elevs.astype(object) != None
            if mask.mean() < 0.2:
                return None, None, None

            valid_lon = points[:, 0][mask]
            valid_lat = points[:, 1][mask]
            valid_z = elevs[mask]

            lon_reg = np.linspace(lon_min, lon_max, resolution)
            lat_reg = np.linspace(lat_min, lat_max, resolution)
            lon_grid, lat_grid = np.meshgrid(lon_reg, lat_reg)

            z_grid = griddata(
                (valid_lon, valid_lat),
                valid_z,
                (lon_grid, lat_grid),
                method="linear",
            )
            if np.isnan(z_grid).any():
                z_grid_nn = griddata(
                    (valid_lon, valid_lat),
                    valid_z,
                    (lon_grid, lat_grid),
                    method="nearest",
                )
                z_grid = np.where(np.isnan(z_grid), z_grid_nn, z_grid)

            self._last_grid_key = grid_key
            self._last_grid_data = (lon_grid, lat_grid, z_grid)
            return lon_grid, lat_grid, z_grid
        finally:
            builtins.print = _orig_print


TERRAIN_CACHE = TerrainCache()


def _get_tile_names(lat_min: float, lat_max: float, lon_min: float, lon_max: float) -> list:
    """Return the list of .hgt tile filenames covering a bounding box.
    
    Tiles are named NxxEyyy.hgt (e.g. N00E006.hgt) for 1-degree SRTM cells.
    The lower-left corner integer determines the tile name.
    """
    tiles = set()
    for lat in range(int(math.floor(lat_min)), int(math.floor(lat_max)) + 1):
        for lon in range(int(math.floor(lon_min)), int(math.floor(lon_max)) + 1):
            ns = "N" if lat >= 0 else "S"
            ew = "E" if lon >= 0 else "W"
            tile = f"{ns}{abs(lat):02d}{ew}{abs(lon):03d}.hgt"
            tiles.add(tile)
    return sorted(tiles)


def _list_cached_tiles() -> list:
    """Return sorted list of .hgt filenames currently in elevation_cache/."""
    if not ELEVATION_CACHE_DIR.exists():
        return []
    return sorted(f.name for f in ELEVATION_CACHE_DIR.glob("*.hgt"))


def _terrain_only_render(lat_min, lat_max, lon_min, lon_max, width_px, height_px, theme_ref, is_dark_ref):
    """Render terrain-only 3D surface as base64 PNG (no scatter points)."""
    import numpy as np
    dpi = 100
    fig_w = width_px / dpi
    fig_h = height_px / dpi
    fig = plt.figure(figsize=(fig_w, fig_h), dpi=dpi)
    ax = fig.add_subplot(111, projection="3d")

    terrain_available = False
    try:
        X, Y, Z = TERRAIN_CACHE.get_elevation_grid(
            lat_min, lat_max, lon_min, lon_max, resolution=80,
        )
        terrain_available = X is not None
    except Exception:
        X = Y = Z = None

    if terrain_available:
        ax.plot_surface(
            X, Y, Z, cmap="gist_earth", alpha=0.85,
            rcount=80, ccount=80, linewidth=0, antialiased=False,
        )
    else:
        ax.set_title("No terrain data available", fontsize=12)

    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_zlabel("Elevation (m)")
    ax.set_title(f"Terrain: {lat_min:.2f} to {lat_max:.2f}, {lon_min:.2f} to {lon_max:.2f}")

    t = theme_ref
    if is_dark_ref:
        ax.set_facecolor("#1c1e21")
        fig.patch.set_facecolor("#1c1e21")
        ax.xaxis.label.set_color("white")
        ax.yaxis.label.set_color("white")
        ax.zaxis.label.set_color("white")
        ax.title.set_color("white")
        ax.tick_params(colors="white")
        ax.xaxis._axinfo["grid"]["color"] = (0.2, 0.2, 0.2, 1)
        ax.yaxis._axinfo["grid"]["color"] = (0.2, 0.2, 0.2, 1)
        ax.zaxis._axinfo["grid"]["color"] = (0.2, 0.2, 0.2, 1)
    else:
        ax.set_facecolor("white")
        fig.patch.set_facecolor("white")

    buf = BytesIO()
    plt.savefig(buf, format="png", bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    buf.seek(0)
    data = buf.read()
    import base64
    b64 = base64.b64encode(data).decode("ascii")
    return f"data:image/png;base64,{b64}"


def _render_3d_scatter(data_list, width_px, height_px, theme_ref, is_dark_ref):
    """Render a 3D terrain + GPS/CO₂ scatter plot as a PNG and return base64 src.
    
    NOTE: This is a pure function (no Flet state access) so it can safely run in a
    background thread via run_in_executor.
    """
    if not data_list:
        return None
    t = theme_ref
    import numpy as np
    dpi = 100
    fig_w = width_px / dpi
    fig_h = height_px / dpi
    fig = plt.figure(figsize=(fig_w, fig_h), dpi=dpi)
    ax = fig.add_subplot(111, projection="3d")

    lats = np.array([d[0] for d in data_list])
    lons = np.array([d[1] for d in data_list])
    co2s = np.array([d[2] for d in data_list])
    alts = np.array([d[3] if len(d) > 3 else 0.0 for d in data_list])

    min_co2, max_co2 = float(co2s.min()), float(co2s.max())
    co2_range = max(max_co2 - min_co2, 1.0)
    sizes = 20 + ((co2s - min_co2) / co2_range) * 80

    # --- Terrain ---
    lat_min, lat_max = float(lats.min()), float(lats.max())
    lon_min, lon_max = float(lons.min()), float(lons.max())
    lat_pad = max((lat_max - lat_min) * 0.15, 0.002)
    lon_pad = max((lon_max - lon_min) * 0.15, 0.002)
    terrain_available = False
    try:
        X, Y, Z = TERRAIN_CACHE.get_elevation_grid(
            lat_min - lat_pad, lat_max + lat_pad,
            lon_min - lon_pad, lon_max + lon_pad,
            resolution=50,
        )
        terrain_available = X is not None
    except Exception:
        X = Y = Z = None

    if terrain_available:
        z_min_surf = float(Z.min())
        ax.plot_surface(
            X, Y, Z,
            cmap="gist_earth",
            alpha=0.85,
            rcount=50, ccount=50,
            linewidth=0, antialiased=False,
        )
        alt_shift = float(alts.min()) - z_min_surf
        shifted_alts = alts - alt_shift
        zlabel = "Altitude above terrain (m)"
    else:
        shifted_alts = alts
        zlabel = "Altitude (m)"

    # --- Connecting trail ---
    ax.plot(lons, lats, shifted_alts, color="gray", alpha=0.35, linewidth=1)

    # --- CO₂ scatter ---
    sc = ax.scatter3D(
        lons, lats, shifted_alts,
        c=co2s,
        s=sizes,
        cmap="coolwarm",
        alpha=0.7,
        marker="D",
        depthshade=False,
    )

    # Colorbar
    cbar = fig.colorbar(sc, ax=ax, label="CO₂ (ppm)", shrink=0.8, aspect=20)

    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_zlabel(zlabel)

    ax.set_title("GPS / CO₂ 3D Terrain View")

    # Theme-aware colors
    if is_dark_ref:
        ax.set_facecolor("#1c1e21")
        fig.patch.set_facecolor("#1c1e21")
        ax.xaxis.label.set_color("white")
        ax.yaxis.label.set_color("white")
        ax.zaxis.label.set_color("white")
        ax.title.set_color("white")
        ax.tick_params(colors="white")
        ax.xaxis._axinfo["grid"]["color"] = (0.2, 0.2, 0.2, 1)
        ax.yaxis._axinfo["grid"]["color"] = (0.2, 0.2, 0.2, 1)
        ax.zaxis._axinfo["grid"]["color"] = (0.2, 0.2, 0.2, 1)
        cbar.ax.yaxis.set_tick_params(colors="white")
        cbar.ax.yaxis.label.set_color("white")
        cbar.outline.set_edgecolor("#555555")
    else:
        ax.set_facecolor("white")
        fig.patch.set_facecolor("white")

    # Watermark if terrain unavailable
    if not terrain_available:
        fig.text(0.5, 0.02, "Terrain unavailable", ha="center", fontsize=10,
                 color="red" if is_dark_ref else "darkred")

    buf = BytesIO()
    plt.savefig(buf, format="png", bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    buf.seek(0)
    data = buf.read()
    import base64
    b64 = base64.b64encode(data).decode("ascii")
    return f"data:image/png;base64,{b64}"


async def main(page: ft.Page):
    page.title = "MiniHawk Dashboard"
    
    current_theme = load_theme()  # "dark" or "light"
    is_dark = current_theme == "dark"
    theme = THEMES[current_theme]

    config = load_config()
    restart_event = asyncio.Event()

    # --- Dirty flags for high-frequency decoupled render loop ---
    dirty_charts = set()
    dirty_gps = [False]
    dirty_rate = [False]
    dirty_conn = [False]
    dirty_table = [False]

    # --- Safe debounced page update for modals / settings / theme toggles ---
    _ui_refresh_pending = [False]

    def schedule_ui_refresh():
        """Queue a page update for dialogs and structural changes safely."""
        if not _ui_refresh_pending[0]:
            _ui_refresh_pending[0] = True
            asyncio.get_event_loop().call_soon(_do_ui_refresh)

    def _do_ui_refresh():
        _ui_refresh_pending[0] = False
        try:
            page.update()
        except Exception:
            pass

    page.theme_mode = ft.ThemeMode.DARK if is_dark else ft.ThemeMode.LIGHT
    page.bgcolor = theme["page_bg"]
    page.padding = ft.Padding.symmetric(horizontal=10, vertical=6)

    # Show a loading indicator immediately so the window doesn't stay blank
    loading = ft.ProgressRing(width=32, height=32, color=theme["text_blue"])
    loading_text = ft.Text("Loading dashboard...", size=14, color=theme["text_secondary"])
    loading_col = ft.Column([loading, loading_text], alignment=ft.MainAxisAlignment.CENTER, horizontal_alignment=ft.CrossAxisAlignment.CENTER)
    loading_container = ft.Container(content=loading_col, expand=True, alignment=ft.alignment.Alignment(0, 0))
    page.add(loading_container)
    schedule_ui_refresh()

    # --- Data State ---
    telemetry_history = []
    co2_pts, hum_pts, temp_pts = [], [], []
    h2s_pts, so2_pts, co2fine_pts = [], [], []
    x_idx = {"CO2": 0, "HUM": 0, "TEMP": 0, "H2S": 0, "SO2": 0, "CO2_FINE": 0}
    gps_state = {"lat": 0.0, "lon": 0.0, "alt": 0.0, "fix_type": 0, "satellites": 0, "eph_cm": 9999}
    _last_scatter_time = [0.0]
    SCATTER_MIN_INTERVAL = 1.5  # seconds

    # Per-sensor rate tracking
    _rate_counts = {"_last_reset": time.time(), "CO2": 0, "HUMIDITY": 0, "TEMP": 0, "H2S": 0, "SO2": 0, "CO2_FINE": 0}
    sample_rate_val = ft.Text("0.0 Hz", size=10, color=theme["label_color"])

    # Connection state: 0 = waiting, 1 = active, -1 = lost
    conn_state = [0]
    conn_dot = ft.Container(
        width=10, height=10, bgcolor=ft.Colors.YELLOW,
        border_radius=5, tooltip="Waiting for MAVLink..."
    )

    # --- Theme application helpers ---
    chart_cards = []           # CO2, HUM, TEMP, scatter
    georef_panel_ref = [None]
    log_container_ref = [None]
    gps_badge_ref = [None]
    pos_badge_ref = [None]
    theme_btn_ref = [None]
    charts_refs = []           # (series, chart, label_color_key)
    scatter_dlg_ref = [None]   # maximized scatter dialog
    line_chart_dlgs = {}       # chart -> maximized dialog ref

    # --- Header UI ---
    pos_badge = ft.Container(
        content=ft.Text("—  —  —", size=10, weight="bold"),
        padding=ft.Padding.symmetric(horizontal=8, vertical=4),
        bgcolor=theme["card_bg"],
        border_radius=5,
    )

    gps_badge = ft.Container(
        content=ft.Text(format_gps_badge(0, 0, 9999), size=10, weight="bold"),
        padding=ft.Padding.all(5),
        bgcolor=theme["gps_quality_bad"],
        border_radius=5,
    )
    gps_badge_ref[0] = gps_badge
    pos_badge_ref[0] = pos_badge

    def apply_chart_themes():
        """Re-render all chart axis colors and grid lines for current theme."""
        t = theme
        grid_color = t["grid_color"]
        label_color = t["label_color"]
        for series, chart in charts_refs:
            chart.horizontal_grid_lines.color = grid_color
            chart.vertical_grid_lines.color = grid_color
            chart.border = ft.Border.all(1, t["axis_color"])
            if chart.left_axis and chart.left_axis.labels:
                for lbl in chart.left_axis.labels:
                    lbl.label.color = label_color
            if chart.bottom_axis and chart.bottom_axis.labels:
                for lbl in chart.bottom_axis.labels:
                    lbl.label.color = label_color

    def render_3d_scatter(data_list, width_px, height_px):
        """Synchronous wrapper for threaded callers inside main()."""
        return _render_3d_scatter(data_list, width_px, height_px, theme, is_dark)

    def update_scatter_plot():
        """Update the scatter plot — always 3D terrain view."""
        t = theme
        n_points = len(gps_co2_data)

        if n_points == 0:
            scatter_img.visible = False
            scatter_placeholder.visible = True
            scatter_box.border = ft.Border.all(1, t["axis_color"])
            try:
                scatter_container.update()
            except Exception:
                pass
            return

        src = render_3d_scatter(gps_co2_data, 900, 820)
        if src:
            scatter_img.src = src
            scatter_img.visible = True
            scatter_placeholder.visible = False
            scatter_box.border = ft.Border.all(1, t["axis_color"])
            try:
                scatter_container.update()
            except Exception:
                pass

    def update_scatter_plot_sync():
        """Synchronous caller — use for theme toggles / dialogs."""
        update_scatter_plot()

    async def _update_scatter_plot_async():
        """Non-blocking: run Matplotlib + SRTM in a thread, then update the widget."""
        t = theme
        data_copy = list(gps_co2_data)
        theme_copy = dict(theme)
        is_dark_copy = bool(is_dark)

        if len(data_copy) == 0:
            scatter_img.visible = False
            scatter_placeholder.visible = True
            try:
                scatter_container.update()
            except Exception:
                pass
            return

        # Run heavy Matplotlib + SRTM in a thread pool
        loop = asyncio.get_event_loop()
        try:
            src = await asyncio.wait_for(
                loop.run_in_executor(
                    None,
                    _render_3d_scatter,
                    data_copy,
                    900,
                    820,
                    theme_copy,
                    is_dark_copy,
                ),
                timeout=10.0,
            )
        except Exception:
            return

        if src:
            scatter_img.src = src
            scatter_img.visible = True
            scatter_placeholder.visible = False
            try:
                scatter_container.update()
            except Exception:
                pass

    # --- Chart factory for grouped line charts ---
    def make_chart(colors, on_event):
        t = theme
        if not isinstance(colors, (list, tuple)):
            colors = [colors]
        series = [
            fch.LineChartData(
                color=color,
                stroke_width=2,
                curved=True,
                rounded_stroke_cap=True,
                points=[],
            )
            for color in colors
        ]
        chart = fch.LineChart(
            data_series=series,
            expand=True,
            min_x=0,
            max_x=MAX_POINTS,
            min_y=-1,
            max_y=1,
            animation=0,
            on_event=on_event,
            horizontal_grid_lines=fch.ChartGridLines(
                interval=0.5, color=t["grid_color"], width=1,
            ),
            vertical_grid_lines=fch.ChartGridLines(
                interval=10, color=t["grid_color"], width=1,
            ),
            left_axis=fch.ChartAxis(
                show_labels=True, label_size=36, labels=[],
            ),
            bottom_axis=fch.ChartAxis(
                show_labels=True, label_size=16,
                labels=[
                    fch.ChartAxisLabel(value=0,          label=ft.Text("0",  size=9, color=t["label_color"])),
                    fch.ChartAxisLabel(value=10,         label=ft.Text("10", size=9, color=t["label_color"])),
                    fch.ChartAxisLabel(value=20,         label=ft.Text("20", size=9, color=t["label_color"])),
                    fch.ChartAxisLabel(value=30,         label=ft.Text("30", size=9, color=t["label_color"])),
                    fch.ChartAxisLabel(value=40,         label=ft.Text("40", size=9, color=t["label_color"])),
                    fch.ChartAxisLabel(value=MAX_POINTS, label=ft.Text("50", size=9, color=t["label_color"])),
                ],
            ),
            border=ft.Border.all(1, t["axis_color"]),
        )
        return series, chart

    # --- Hover handlers ---
    def make_on_event(georef_list: list):
        def handler(e: fch.LineChartEvent):
            try:
                if not e.spots:
                    return
                spot = e.spots[0]
                idx = spot.spot_index if hasattr(spot, "spot_index") else spot[1]
                if 0 <= idx < len(georef_list):
                    d = georef_list[idx]
                    georef_val.value   = f"{d['val']:.4f}"
                    georef_val.color   = theme["text_primary"]
                    georef_lat.value   = f"{d['lat']:.5f}°"
                    georef_lon.value   = f"{d['lon']:.5f}°"
                    georef_alt.value   = f"↑ {d['alt']:.1f}m"
                    georef_panel_ref[0].update()
            except Exception:
                pass
        return handler

    def make_group_on_event(georef_lists: list[list]):
        """Show GPS data for the hovered series; older Flet versions fall back to the first one."""
        def handler(e: fch.LineChartEvent):
            try:
                if not e.spots:
                    return
                spot = e.spots[0]
                point_index = spot.spot_index if hasattr(spot, "spot_index") else spot[1]
                series_index = next(
                    (getattr(spot, attr) for attr in ("series_index", "bar_index", "data_set_index")
                     if hasattr(spot, attr)),
                    0,
                )
                georef_list = georef_lists[int(series_index)] if 0 <= int(series_index) < len(georef_lists) else georef_lists[0]
                if 0 <= point_index < len(georef_list):
                    d = georef_list[point_index]
                    georef_val.value = f"{d['val']:.4f}"
                    georef_val.color = theme["text_primary"]
                    georef_lat.value = f"{d['lat']:.5f}°"
                    georef_lon.value = f"{d['lon']:.5f}°"
                    georef_alt.value = f"↑ {d['alt']:.1f}m"
                    georef_panel_ref[0].update()
            except Exception:
                pass
        return handler

    # --- Async scatter render task ---
    _scatter_render_task = [None]

    co2_georef: list = []
    hum_georef: list = []
    temp_georef: list = []
    h2s_georef: list = []
    so2_georef: list = []
    co2fine_georef: list = []

    # Tres paneles: pares con escalas comparables comparten el mismo eje vertical.
    climate_series, climate_chart = make_chart(
        [ft.Colors.GREEN_400, ft.Colors.ORANGE_400], make_group_on_event([hum_georef, temp_georef])
    )
    gas_series, gas_chart = make_chart(
        [ft.Colors.YELLOW_400, ft.Colors.RED_400], make_group_on_event([h2s_georef, so2_georef])
    )
    co2_series, co2_chart = make_chart(
        [ft.Colors.PURPLE_400, ft.Colors.CYAN_400], make_group_on_event([co2_georef, co2fine_georef])
    )
    hum_series, hum_chart = make_chart(ft.Colors.GREEN_400, make_on_event(hum_georef))
    temp_series, temp_chart = make_chart(ft.Colors.ORANGE_400, make_on_event(temp_georef))
    h2s_series, h2s_chart = make_chart(ft.Colors.YELLOW_400, make_on_event(h2s_georef))
    so2_series, so2_chart = make_chart(ft.Colors.RED_400, make_on_event(so2_georef))
    co2fine_series, co2fine_chart = make_chart(ft.Colors.CYAN_400, make_on_event(co2fine_georef))
    # Dedicated CO2 panel used in the individual view.
    co2_individual_series, co2_individual_chart = make_chart(ft.Colors.PURPLE_400, make_on_event(co2_georef))
    detail_series, detail_chart = make_chart(ft.Colors.PURPLE_400, make_on_event(co2_georef))

    charts_refs.extend([
        (climate_series, climate_chart),
        (gas_series, gas_chart),
        (co2_series, co2_chart),
        (hum_series, hum_chart),
        (temp_series, temp_chart),
        (h2s_series, h2s_chart),
        (so2_series, so2_chart),
        (co2fine_series, co2fine_chart),
        (co2_individual_series, co2_individual_chart),
        (detail_series, detail_chart),
    ])

    def chart_card(label, color, chart, maximize_action=None, config_action=None, on_tap=None):
        t = theme
        config_buttons = []
        if callable(config_action):
            config_buttons.append(ft.IconButton(
                icon=ft.Icons.TUNE,
                icon_size=14,
                tooltip="Configure bands",
                on_click=config_action,
                padding=ft.Padding.all(4),
                icon_color=color,
            ))
        elif config_action:
            for sensor_label, sensor_color, action in config_action:
                config_buttons.append(ft.IconButton(
                    icon=ft.Icons.TUNE,
                    icon_size=14,
                    tooltip=f"Configure bands: {sensor_label}",
                    on_click=action,
                    padding=ft.Padding.all(4),
                    icon_color=sensor_color,
                ))

        title_controls = [
            ft.Text(label, size=11, weight="bold", color=color, expand=True),
            *config_buttons,
        ]
        if on_tap:
            title_controls.append(ft.IconButton(
                icon=ft.Icons.OPEN_IN_NEW,
                icon_size=14,
                tooltip="Open detailed chart",
                on_click=on_tap,
                padding=ft.Padding.all(4),
                icon_color=color,
            ))
        if maximize_action:
            title_controls.append(ft.IconButton(
                icon=ft.Icons.FULLSCREEN,
                icon_size=14,
                tooltip="Maximize",
                on_click=maximize_action,
                padding=ft.Padding.all(4),
            ))
        title_row = ft.Row(title_controls, tight=True)
        c = ft.Container(
            content=ft.Column([
                title_row,
                chart,
            ], spacing=0, expand=True),
            bgcolor=t["card_bg"],
            padding=ft.Padding.symmetric(horizontal=10, vertical=8),
            border_radius=10,
            expand=True,
            on_click=on_tap,
        )
        chart_cards.append(c)
        return c

    # --- Maximize chart dialog factory ---
    def create_maximize_handler(label, color, chart, georef):
        def close_dlg(e):
            dlg = line_chart_dlgs.pop(chart, None)
            if dlg:
                dlg.open = False
            schedule_ui_refresh()

        def handler(e):
            live_series = list(chart.data_series or [])
            dlg_chart = fch.LineChart(
                data_series=[
                    fch.LineChartData(
                        color=source.color,
                        stroke_width=3,
                        curved=True,
                        rounded_stroke_cap=True,
                        points=list(source.points),
                    )
                    for source in live_series
                ],
                expand=True,
                min_x=chart.min_x,
                max_x=chart.max_x,
                min_y=chart.min_y,
                max_y=chart.max_y,
                animation=0,
                on_event=chart.on_event,
                horizontal_grid_lines=fch.ChartGridLines(
                    interval=chart.horizontal_grid_lines.interval,
                    color=theme["grid_color"], width=1,
                ),
                vertical_grid_lines=fch.ChartGridLines(
                    interval=chart.vertical_grid_lines.interval,
                    color=theme["grid_color"], width=1,
                ),
                left_axis=fch.ChartAxis(show_labels=True, label_size=36, labels=chart.left_axis.labels),
                bottom_axis=fch.ChartAxis(show_labels=True, label_size=16, labels=chart.bottom_axis.labels),
                border=ft.Border.all(1, theme["axis_color"]),
            )
            dlg = ft.AlertDialog(
                modal=True,
                title=ft.Text(label, size=16, weight="bold", color=color),
                content=ft.Container(
                    content=ft.Column([dlg_chart], expand=True),
                    width=700,
                    height=400,
                    border=ft.Border.all(1, theme["axis_color"]),
                    border_radius=10,
                ),
                bgcolor=theme["card_bg"],
                actions=[ft.TextButton("Close", on_click=close_dlg)],
                actions_alignment=ft.MainAxisAlignment.END,
            )
            page.overlay.append(dlg)
            line_chart_dlgs[chart] = dlg
            dlg.open = True
            schedule_ui_refresh()
        return handler

    # --- Georef panel ---
    georef_val = ft.Text("—", size=12, weight="bold", color=theme["text_primary"])
    georef_lat = ft.Text("—", size=12, color=theme["text_blue"])
    georef_lon = ft.Text("—", size=12, color=theme["text_blue"])
    georef_alt = ft.Text("—", size=12, color=theme["text_green"])

    georef_panel = ft.Container(
        content=ft.Row([
            ft.Text("Hover:",  size=11, color=theme["label_color"]),
            ft.Text("Val",     size=11, color=theme["label_color"]), georef_val,
            ft.Text("Lat",     size=11, color=theme["label_color"]), georef_lat,
            ft.Text("Lon",     size=11, color=theme["label_color"]), georef_lon,
            ft.Text("Alt",     size=11, color=theme["label_color"]), georef_alt,
        ], spacing=6, tight=True),
        bgcolor=theme["card_bg"],
        padding=ft.Padding.symmetric(horizontal=14, vertical=8),
        border_radius=8,
        border=ft.Border.all(1, theme["georef_border"]),
    )
    georef_panel_ref[0] = georef_panel

    # --- Scatter ---
    gps_co2_data = []
    scatter_img = ft.Image(src="", fit="contain", expand=True, visible=False)
    scatter_placeholder = ft.Container(
        content=ft.Text("No data", color=theme["scatter_no_data"]),
        expand=True,
        alignment=ft.alignment.Alignment.CENTER,
    )
    scatter_box = ft.Container(
        content=ft.Stack([scatter_placeholder, scatter_img], expand=True),
        alignment=ft.alignment.Alignment.CENTER,
        expand=True,
        border=ft.Border.all(1, theme["axis_color"]),
    )
    def create_scatter_maximize_handler():
        def close_dlg(e):
            if scatter_dlg_ref[0]:
                scatter_dlg_ref[0].open = False
            scatter_dlg_ref[0] = None
            schedule_ui_refresh()

        def handler(e):
            t = theme
            if len(gps_co2_data) == 0:
                return
            src = render_3d_scatter(gps_co2_data, 1100, 620)
            if not src:
                return
            img = ft.Image(src=src, fit="contain", expand=True)
            scatter_dlg_ref[0] = img
            dlg = ft.AlertDialog(
                modal=True,
                title=ft.Text("GPS/CO2 3D Terrain", size=16, weight="bold", color=t["text_cyan"]),
                content=ft.Container(
                    content=img,
                    width=1160,
                    height=660,
                    alignment=ft.alignment.Alignment.CENTER,
                    border=ft.Border.all(1, t["axis_color"]),
                    border_radius=10,
                ),
                bgcolor=t["card_bg"],
                actions=[ft.TextButton("Close", on_click=close_dlg)],
                actions_alignment=ft.MainAxisAlignment.END,
            )
            page.overlay.append(dlg)
            scatter_dlg_ref[0] = dlg
            dlg.open = True
            schedule_ui_refresh()
        return handler

    scatter_container = ft.Container(
        content=ft.Column([
            ft.Row([
                ft.Text("GPS/CO2 3D Terrain", size=11, weight="bold", color=theme["text_cyan"], expand=True),
                ft.IconButton(
                    icon=ft.Icons.FULLSCREEN,
                    icon_size=14,
                    tooltip="Maximize",
                    on_click=create_scatter_maximize_handler(),
                    padding=ft.Padding.all(4),
                ),
            ], tight=True),
            scatter_box,
        ], spacing=5, expand=True),
        bgcolor=theme["card_bg"],
        padding=15,
        border_radius=10,
        expand=True,
    )
    chart_cards.append(scatter_container)


    # --- Log (moved to a dialog window) ---
    log_entries = deque(maxlen=200)
    log_view = ft.ListView(expand=True, spacing=4, auto_scroll=True)

    def populate_log_view():
        log_view.controls = [
            ft.Text(msg, size=10, color=col)
            for msg, col in log_entries
        ]

    def add_log_message(msg: str, color=None):
        col = color if color is not None else theme["text_secondary"]
        log_entries.append((msg, col))
        if getattr(log_dialog, "open", False):
            log_view.controls.append(ft.Text(msg, size=10, color=col))
            if len(log_view.controls) > 120:
                log_view.controls.pop(0)
            try:
                log_view.update()
            except Exception:
                pass

    log_dialog = ft.AlertDialog(
        modal=True,
        title=ft.Text("Telemetry Logs", color=theme["text_primary"], weight="bold"),
        content=ft.Container(
            content=log_view,
            width=800,
            height=400,
            bgcolor=theme["log_bg"],
            padding=10,
            border_radius=5,
        ),
        bgcolor=theme["card_bg"],
        actions=[
            ft.TextButton("Close", on_click=lambda e: [setattr(log_dialog, "open", False), schedule_ui_refresh()]),
        ],
        actions_alignment=ft.MainAxisAlignment.END,
    )
    
    # Add the dialog overlay so it can actually render
    page.overlay.append(log_dialog)

    # --- Log button ---
    def toggle_log_dialog(e):
        populate_log_view()
        log_dialog.open = True
        schedule_ui_refresh()

    log_btn = ft.IconButton(
        icon=ft.Icons.TERMINAL,
        tooltip="Open Logs",
        on_click=toggle_log_dialog,
    )

    # --- Raw Data Table (persistent dashboard tab) ---
    def build_data_table():
        columns = [
            ft.DataColumn(ft.Text("Timestamp", size=10, color=theme["text_primary"])),
            ft.DataColumn(ft.Text("Hora", size=10, color=theme["text_primary"])),
            ft.DataColumn(ft.Text("Sensor", size=10, color=theme["text_primary"])),
            ft.DataColumn(ft.Text("Value", size=10, color=theme["text_primary"])),
            ft.DataColumn(ft.Text("Lat", size=10, color=theme["text_primary"])),
            ft.DataColumn(ft.Text("Lon", size=10, color=theme["text_primary"])),
            ft.DataColumn(ft.Text("Alt", size=10, color=theme["text_primary"])),
        ]
        rows = []
        for row in telemetry_history[-250:]:
            timestamp = row["timestamp"]
            rows.append(
                ft.DataRow(
                    cells=[
                        ft.DataCell(ft.Text(timestamp, size=9, color=theme["text_secondary"])),
                        ft.DataCell(ft.Text(timestamp.split(" ")[-1], size=9, color=theme["text_secondary"])),
                        ft.DataCell(ft.Text(row["name"], size=9, color=theme["text_secondary"])),
                        ft.DataCell(ft.Text(f"{row['value']:.3f}", size=9, color=theme["text_secondary"])),
                        ft.DataCell(ft.Text(f"{row['lat']:.5f}", size=9, color=theme["text_secondary"])),
                        ft.DataCell(ft.Text(f"{row['lon']:.5f}", size=9, color=theme["text_secondary"])),
                        ft.DataCell(ft.Text(f"{row['alt']:.1f}", size=9, color=theme["text_secondary"])),
                    ]
                )
            )
        return ft.DataTable(
            columns=columns,
            rows=rows,
            border=ft.Border.all(1, theme["axis_color"]),
            border_radius=4,
            vertical_lines=ft.border.BorderSide(1, theme["grid_color"]),
            horizontal_lines=ft.border.BorderSide(1, theme["grid_color"]),
            heading_row_color=ft.Colors.with_opacity(0.1, theme["label_color"]),
        )

    def show_band_config(sensor_key):
        """Open a dialog to configure color bands for a sensor."""
        t = theme
        use_bands = config.get("use_bands", {}).get(sensor_key, False)
        band_list = config.get("bands", {}).get(sensor_key, DEFAULT_CONFIG["bands"].get(sensor_key, []))
        band_rows = []  # list of control lists for display
        band_inputs = []  # list of dicts with field refs
        available_colors = list(COLOR_MAP.keys())

        def add_band_row(band_data, removable=True):
            max_field = ft.TextField(
                value=str(band_data.get("max", 999999)), label="Max", width=80,
                text_size=12, keyboard_type=ft.KeyboardType.NUMBER,
            )
            color_dd = ft.Dropdown(
                value=band_data.get("color", "green"), width=110, text_size=12,
                options=[ft.dropdown.Option(c) for c in available_colors],
            )
            label_field = ft.TextField(
                value=band_data.get("label", ""), label="Label", width=120, text_size=12,
            )
            ctrls = [max_field, color_dd, label_field]
            if removable:
                def remove_row(e, target=ctrls):
                    for i, c in enumerate(band_rows):
                        if c is target:
                            band_rows.pop(i)
                            band_inputs.pop(i)
                            refresh_band_rows()
                            break
                ctrls.append(ft.IconButton(
                    icon=ft.Icons.DELETE, icon_size=16, tooltip="Remove band",
                    on_click=remove_row,
                ))
            band_rows.append(ctrls)
            band_inputs.append({"max": max_field, "color": color_dd, "label": label_field})

        def refresh_band_rows():
            header = ft.Row([
                ft.Text("Max", size=10, color=t["label_color"], width=80),
                ft.Text("Color", size=10, color=t["label_color"], width=110),
                ft.Text("Label", size=10, color=t["label_color"], width=120),
            ])
            bands_col.controls = [header] + [
                ft.Row(ctrls, spacing=8, vertical_alignment=ft.CrossAxisAlignment.CENTER)
                for ctrls in band_rows
            ]
            schedule_ui_refresh()

        for band in band_list:
            add_band_row(band)

        bands_col = ft.Column(spacing=6, scroll=ft.ScrollMode.AUTO, height=280)
        refresh_band_rows()

        use_bands_toggle = ft.Switch(
            value=use_bands,
            label="Use static bands (instead of dynamic intensity)",
            active_color=t["text_cyan"],
        )

        def add_new_band(e):
            add_band_row({"max": 999999, "color": "green", "label": "New"})
            refresh_band_rows()

        def save_bands(e):
            new_bands = []
            for inputs in band_inputs:
                try:
                    max_val = float(inputs["max"].value)
                except ValueError:
                    max_val = 999999
                new_bands.append({
                    "max": max_val,
                    "color": inputs["color"].value,
                    "label": inputs["label"].value,
                })
            new_bands.sort(key=lambda b: b["max"])
            config.setdefault("use_bands", {})
            config.setdefault("bands", {})
            config["use_bands"][sensor_key] = use_bands_toggle.value
            config["bands"][sensor_key] = new_bands
            save_config(config)
            if sensor_key == "SCATTER":
                update_scatter_plot()
            else:
                # trigger chart refresh by re-rendering latest point
                schedule_ui_refresh()
            dlg.open = False
            schedule_ui_refresh()

        dlg = ft.AlertDialog(
            modal=True,
            title=ft.Text(f"Bands: {sensor_key}", size=14, weight="bold", color=t["text_primary"]),
            content=ft.Column([
                use_bands_toggle,
                ft.Divider(height=1, color=t["grid_color"]),
                bands_col,
                ft.FilledButton("Add Band", icon=ft.Icons.ADD, on_click=add_new_band),
            ], spacing=10, tight=True),
            bgcolor=t["card_bg"],
            actions=[
                ft.TextButton("Cancel", on_click=lambda e: [setattr(dlg, "open", False), schedule_ui_refresh()]),
                ft.TextButton("Save", on_click=save_bands),
            ],
            actions_alignment=ft.MainAxisAlignment.END,
        )
        page.overlay.append(dlg)
        dlg.open = True
        schedule_ui_refresh()

    # --- File manager helper ---
    def _open_file_manager(path: Path):
        try:
            if sys.platform == "win32":
                subprocess.Popen(["explorer", "/select,", str(path)])
            elif sys.platform == "darwin":
                subprocess.Popen(["open", "-R", str(path)])
            else:
                subprocess.Popen(["xdg-open", str(path.parent)])
        except Exception:
            pass

    # Instantiate FilePicker but do NOT add to overlay (avoids "Unknown control" error)
    file_picker = ft.FilePicker()

    async def download_csv(e):
        import io

        # Build CSV in memory
        output = io.StringIO()
        writer = csv.DictWriter(output, fieldnames=["name", "value", "lat", "lon", "alt"])
        writer.writeheader()
        writer.writerows(telemetry_history)
        csv_bytes = output.getvalue().encode("utf-8")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"sensors_export_{timestamp}.csv"

        platform = page.platform or ""
        if platform in ("android", "ios"):
            # Mobile: trigger native share sheet
            try:
                share = ft.Share()
                await share.share_files(
                    files=[
                        ft.ShareFile(
                            data=csv_bytes,
                            mime_type="text/csv",
                            name=filename,
                        )
                    ],
                    text="MiniHawk Telemetry Export",
                )
                page.snack_bar = ft.SnackBar(ft.Text("CSV shared successfully"), duration=3000)
                page.snack_bar.open = True
            except Exception as ex:
                page.snack_bar = ft.SnackBar(ft.Text(f"Share error: {ex}"))
                page.snack_bar.open = True
            schedule_ui_refresh()
        else:
            # Desktop: file picker dialog (NOT added to overlay)
            try:
                save_path = await file_picker.save_file(
                    dialog_title="Save Telemetry CSV",
                    file_name=filename,
                )
                if save_path:
                    if not save_path.endswith(".csv"):
                        save_path += ".csv"
                    with open(save_path, "wb") as f:
                        f.write(csv_bytes)
                    page.snack_bar = ft.SnackBar(
                        ft.Text(f"Saved to {save_path}"),
                        duration=4000,
                    )
                    page.snack_bar.open = True
                    schedule_ui_refresh()
                    _open_file_manager(Path(save_path))
                else:
                    # User cancelled
                    page.snack_bar = ft.SnackBar(ft.Text("Save cancelled"), duration=3000)
                    page.snack_bar.open = True
                    schedule_ui_refresh()
            except Exception as ex:
                # FilePicker unavailable or failed — fallback to Downloads
                try:
                    downloads_dir = Path.home() / "Downloads"
                    downloads_dir.mkdir(parents=True, exist_ok=True)
                    save_path = downloads_dir / filename
                    with open(save_path, "wb") as f:
                        f.write(csv_bytes)
                    page.snack_bar = ft.SnackBar(
                        ft.Text(f"Saved to {save_path}"),
                        duration=4000,
                    )
                    page.snack_bar.open = True
                    schedule_ui_refresh()
                    _open_file_manager(save_path)
                except Exception as ex2:
                    page.snack_bar = ft.SnackBar(ft.Text(f"Save error: {ex2}"))
                    page.snack_bar.open = True
                    schedule_ui_refresh()

    # --- Theme Toggle ---
    def toggle_theme(e):
        nonlocal is_dark, theme
        is_dark = not is_dark
        theme_key = "dark" if is_dark else "light"
        theme = THEMES[theme_key]
        save_theme(theme_key)

        page.theme_mode = ft.ThemeMode.DARK if is_dark else ft.ThemeMode.LIGHT
        page.bgcolor = theme["page_bg"]

        for c in chart_cards:
            c.bgcolor = theme["card_bg"]

        georef_panel.bgcolor = theme["card_bg"]
        georef_panel.border = ft.Border.all(1, theme["georef_border"])
        georef_val.color = theme["text_primary"]
        pos_badge.bgcolor = theme["card_bg"]

        # Update chart styles
        apply_chart_themes()
        update_scatter_plot()
        refresh_table_view()

        # Update theme button icon
        btn = theme_btn_ref[0]
        if btn:
            btn.icon = ft.Icons.DARK_MODE if is_dark else ft.Icons.LIGHT_MODE
            btn.tooltip = "Switch to Light Mode" if is_dark else "Switch to Dark Mode"

        schedule_ui_refresh()

    theme_btn = ft.IconButton(
        icon=ft.Icons.DARK_MODE if is_dark else ft.Icons.LIGHT_MODE,
        tooltip="Switch to Light Mode" if is_dark else "Switch to Dark Mode",
        on_click=toggle_theme,
    )
    theme_btn_ref[0] = theme_btn

    # --- Settings dialog ---
    async def show_settings(e):
        platform = page.platform or ""
        is_mobile = platform in ("android", "ios")
        bind_field = ft.TextField(
            label="Bind address (your IP)",
            value=config.get("mavlink_bind_host", "0.0.0.0"),
            hint_text="0.0.0.0 = any interface. For WiFi use your WiFi IP (e.g. 172.16.101.180)." if is_mobile else "0.0.0.0 = listen on all interfaces",
        )
        port_field = ft.TextField(
            label="MAVLink UDP port",
            value=str(config.get("mavlink_port", "14550")),
            hint_text="Set sender target to THIS IP : THIS PORT",
            keyboard_type=ft.KeyboardType.NUMBER if hasattr(ft, "KeyboardType") else None,
        )
        async def save_settings(e2):
            bind = bind_field.value.strip()
            port = port_field.value.strip()
            if not bind:
                bind_field.error_text = "Required"
                bind_field.update()
                return
            if not port.isdigit():
                port_field.error_text = "Must be a number"
                port_field.update()
                return
            config["mavlink_bind_host"] = bind
            config["mavlink_port"] = int(port)
            save_config(config)
            dlg.open = False
            schedule_ui_refresh()
            # Restart MAVLink task with new address
            restart_event.set()

        dlg = ft.AlertDialog(
            modal=True,
            title=ft.Text("Settings", size=16, weight="bold", color=theme["text_primary"]),
            content=ft.Column([bind_field, port_field], tight=True),
            actions=[
                ft.TextButton("Cancel", on_click=lambda e2: [setattr(dlg, "open", False), schedule_ui_refresh()]),
                ft.FilledButton("Save", on_click=save_settings),
            ],
            actions_alignment=ft.MainAxisAlignment.END,
            bgcolor=theme["card_bg"],
        )
        page.overlay.append(dlg)
        dlg.open = True
        schedule_ui_refresh()

    settings_btn = ft.IconButton(
        icon=ft.Icons.SETTINGS,
        tooltip="MAVLink settings",
        on_click=show_settings,
    )

    # --- Offline terrain download dialog ---
    def show_offline_download_dialog(e):
        t = theme
        lat_min_field = ft.TextField(
            label="Lat Min", value="", text_size=12, keyboard_type=ft.KeyboardType.NUMBER,
            hint_text="e.g. -23.5",
        )
        lat_max_field = ft.TextField(
            label="Lat Max", value="", text_size=12, keyboard_type=ft.KeyboardType.NUMBER,
            hint_text="e.g. -22.0",
        )
        lon_min_field = ft.TextField(
            label="Lon Min", value="", text_size=12, keyboard_type=ft.KeyboardType.NUMBER,
            hint_text="e.g. -47.0",
        )
        lon_max_field = ft.TextField(
            label="Lon Max", value="", text_size=12, keyboard_type=ft.KeyboardType.NUMBER,
            hint_text="e.g. -45.0",
        )

        progress_bar = ft.ProgressBar(value=0, width=300, visible=False)
        status_text = ft.Text("", size=11, color=t["text_secondary"])
        preview_img = ft.Image(src="", fit="contain", expand=True, visible=False)
        tiles_list = ft.ListView(expand=True, height=180, spacing=2)
        cancel_flag = {"do_cancel": False}

        def refresh_cached_list():
            cached = _list_cached_tiles()
            if cached:
                tile_controls = [
                    ft.Text(f"📦 {tile_name}", size=11, color=t["text_secondary"])
                    for tile_name in cached
                ]
                tiles_list.controls = tile_controls
                tiles_list.visible = True
            else:
                tiles_list.controls = [ft.Text("No cached tiles", size=11, color=t["text_secondary"])]
                tiles_list.visible = True
            schedule_ui_refresh()

        refresh_cached_list()

        def fill_from_gps(_):
            if gps_co2_data:
                lats = [d[0] for d in gps_co2_data]
                lons = [d[1] for d in gps_co2_data]
                margin = max((max(lats) - min(lats)) * 0.2, (max(lons) - min(lons)) * 0.2, 0.01)
                lat_min_field.value = f"{min(lats) - margin:.4f}"
                lat_max_field.value = f"{max(lats) + margin:.4f}"
                lon_min_field.value = f"{min(lons) - margin:.4f}"
                lon_max_field.value = f"{max(lons) + margin:.4f}"
            else:
                status_text.value = "No GPS data available to pre-fill"
            schedule_ui_refresh()

        def nudge_field(field, delta):
            try:
                val = float(field.value) if field.value else 0.0
            except ValueError:
                val = 0.0
            field.value = f"{val + delta:.4f}"
            schedule_ui_refresh()

        def build_nudge_row(field):
            return ft.Row([
                ft.IconButton(
                    icon=ft.Icons.REMOVE, icon_size=12, padding=ft.Padding.all(2),
                    on_click=lambda _: nudge_field(field, -1.0),
                    tooltip="-1°",
                ),
                ft.IconButton(
                    icon=ft.Icons.REMOVE, icon_size=12, padding=ft.Padding.all(2),
                    on_click=lambda _: nudge_field(field, -0.1),
                    tooltip="-0.1°",
                ),
                field,
                ft.IconButton(
                    icon=ft.Icons.ADD, icon_size=12, padding=ft.Padding.all(2),
                    on_click=lambda _: nudge_field(field, 0.1),
                    tooltip="+0.1°",
                ),
                ft.IconButton(
                    icon=ft.Icons.ADD, icon_size=12, padding=ft.Padding.all(2),
                    on_click=lambda _: nudge_field(field, 1.0),
                    tooltip="+1°",
                ),
            ], spacing=4, vertical_alignment=ft.CrossAxisAlignment.CENTER)

        async def start_download(e2):
            try:
                lat_min = float(lat_min_field.value)
                lat_max = float(lat_max_field.value)
                lon_min = float(lon_min_field.value)
                lon_max = float(lon_max_field.value)
            except (ValueError, TypeError):
                status_text.value = "Invalid coordinate values"
                schedule_ui_refresh()
                return

            if lat_min >= lat_max or lon_min >= lon_max:
                status_text.value = "Min must be less than Max for lat and lon"
                schedule_ui_refresh()
                return

            tiles = _get_tile_names(lat_min, lat_max, lon_min, lon_max)
            if not tiles:
                status_text.value = "No SRTM tiles found for this region"
                schedule_ui_refresh()
                return

            progress_bar.visible = True
            progress_bar.value = 0
            status_text.value = f"Preparing to download {len(tiles)} tile(s)..."
            schedule_ui_refresh()

            cache_dir = str(TERRAIN_CACHE.cache_dir)
            results = []
            total = len(tiles)
            cancel_flag["do_cancel"] = False

            for i, tile in enumerate(tiles):
                if cancel_flag["do_cancel"]:
                    status_text.value = f"Cancelled after {i}/{total} tiles"
                    schedule_ui_refresh()
                    break

                tile_file = TERRAIN_CACHE.cache_dir / tile
                if tile_file.exists():
                    results.append((tile, "cached"))
                    status_text.value = f"[{i + 1}/{total}] {tile} — already cached"
                else:
                    status_text.value = f"[{i + 1}/{total}] Downloading {tile}..."
                    progress_bar.value = i / total
                    schedule_ui_refresh()

                    ns = 1 if tile[0] == "N" else -1
                    ew = 1 if tile[3] == "E" else -1
                    center_lat = ns * (int(tile[1:3]) + 0.5)
                    center_lon = ew * (int(tile[4:7]) + 0.5)

                    try:
                        loop = asyncio.get_event_loop()
                        elev = await asyncio.wait_for(
                            loop.run_in_executor(
                                None,
                                lambda lat=center_lat, lon=center_lon: TERRAIN_CACHE.elev.get_elevation(lat, lon),
                            ),
                            timeout=30.0,
                        )
                        results.append((tile, "downloaded"))
                    except Exception:
                        results.append((tile, "failed"))

                progress_bar.value = (i + 1) / total
                schedule_ui_refresh()

            progress_bar.value = 1.0
            new_dl = sum(1 for _, s in results if s == "downloaded")
            cached = sum(1 for _, s in results if s == "cached")
            failed = sum(1 for _, s in results if s == "failed")
            status_text.value = f"{new_dl} downloaded, {cached} cached, {failed} failed"
            refresh_cached_list()

            if results:
                theme_copy = dict(theme)
                is_dark_copy = bool(is_dark)
                try:
                    src = await asyncio.wait_for(
                        loop.run_in_executor(
                            None,
                            _terrain_only_render,
                            lat_min, lat_max, lon_min, lon_max,
                            500, 350, theme_copy, is_dark_copy,
                        ),
                        timeout=15.0,
                    )
                    if src:
                        preview_img.src = src
                        preview_img.visible = True
                        schedule_ui_refresh()
                except Exception:
                    pass

        def cancel_download(e3):
            cancel_flag["do_cancel"] = True

        download_btn = ft.FilledButton("Download Tiles", icon=ft.Icons.DOWNLOAD, on_click=start_download)
        cancel_btn = ft.TextButton("Cancel", on_click=cancel_download)

        dlg = ft.AlertDialog(
            modal=True,
            title=ft.Text("Offline Terrain Download", size=16, weight="bold", color=t["text_primary"]),
            content=ft.Column([
                ft.Text("Select a bounding box to pre-cache SRTM terrain tiles.", size=12, color=t["text_secondary"]),
                ft.Row([
                    build_nudge_row(lat_min_field),
                    build_nudge_row(lat_max_field),
                ], spacing=12),
                ft.Row([
                    build_nudge_row(lon_min_field),
                    build_nudge_row(lon_max_field),
                ], spacing=12),
                ft.Row([
                    ft.TextButton("Fill from GPS data", icon=ft.Icons.MY_LOCATION, on_click=fill_from_gps),
                ]),
                ft.Row([download_btn, cancel_btn, status_text], spacing=12, vertical_alignment=ft.CrossAxisAlignment.CENTER),
                progress_bar,
                ft.Container(
                    content=preview_img,
                    border=ft.Border.all(1, t["axis_color"]),
                    border_radius=ft.BorderRadius.all(4),
                    height=250,
                ),
                tiles_list,
            ], spacing=10, scroll=ft.ScrollMode.AUTO, height=600),
            actions=[
                ft.TextButton("Close", on_click=lambda _: [setattr(dlg, "open", False), schedule_ui_refresh()]),
            ],
            actions_alignment=ft.MainAxisAlignment.END,
            bgcolor=t["card_bg"],
        )
        page.overlay.append(dlg)
        dlg.open = True
        schedule_ui_refresh()

    offline_btn = ft.IconButton(
        icon=ft.Icons.DOWNLOAD,
        tooltip="Download terrain for offline use",
        on_click=show_offline_download_dialog,
    )

    detail_selection = ["CO2"]
    dashboard_tab_index = [0]
    detail_heading = ft.Text("CO₂ (ppm)", size=16, weight="bold", color=ft.Colors.PURPLE_400)

    sensor_view_meta = {
        "CO2": ("CO₂ (ppm)", ft.Colors.PURPLE_400, co2_georef),
        "CO2_FINE": ("CO₂ Fine (ppm)", ft.Colors.CYAN_400, co2fine_georef),
        "HUMIDITY": ("HUMIDITY (%)", ft.Colors.GREEN_400, hum_georef),
        "TEMPERATURE": ("TEMPERATURE (°C)", ft.Colors.ORANGE_400, temp_georef),
        "H2S": ("H2S (ppm)", ft.Colors.YELLOW_400, h2s_georef),
        "SO2": ("SO₂ (ppm)", ft.Colors.RED_400, so2_georef),
    }

    def open_sensor_detail(sensor_name):
        def handler(e):
            label, color, georef = sensor_view_meta[sensor_name]
            detail_selection[0] = sensor_name
            dashboard_tab_index[0] = 1
            detail_heading.value = label
            detail_heading.color = color
            detail_chart.on_event = make_on_event(georef)
            chart_sensor_names[id(detail_chart)] = [sensor_name]
            _rebuild_chart_series(detail_chart)
            dashboard_tabs.selected_index = 1
            try:
                dashboard_tabs.update()
            except Exception:
                schedule_ui_refresh()
        return handler

    individual_cards = [
        chart_card("CO₂ (ppm)", ft.Colors.PURPLE_400, co2_individual_chart,
                   create_maximize_handler("CO₂ (ppm)", ft.Colors.PURPLE_400, co2_individual_chart, co2_georef),
                   [("CO₂", ft.Colors.PURPLE_400, lambda e: show_band_config("CO2"))],
                   on_tap=open_sensor_detail("CO2")),
        chart_card("CO₂ Fine (ppm)", ft.Colors.CYAN_400, co2fine_chart,
                   create_maximize_handler("CO₂ Fine (ppm)", ft.Colors.CYAN_400, co2fine_chart, co2fine_georef),
                   [("CO₂ Fine", ft.Colors.CYAN_400, lambda e: show_band_config("CO2_FINE"))],
                   on_tap=open_sensor_detail("CO2_FINE")),
        chart_card("HUMIDITY (%)", ft.Colors.GREEN_400, hum_chart,
                   create_maximize_handler("Humidity (%)", ft.Colors.GREEN_400, hum_chart, hum_georef),
                   [("Humidity", ft.Colors.GREEN_400, lambda e: show_band_config("HUMIDITY"))],
                   on_tap=open_sensor_detail("HUMIDITY")),
        chart_card("TEMPERATURE (°C)", ft.Colors.ORANGE_400, temp_chart,
                   create_maximize_handler("Temperature (°C)", ft.Colors.ORANGE_400, temp_chart, temp_georef),
                   [("Temperature", ft.Colors.ORANGE_400, lambda e: show_band_config("TEMPERATURE"))],
                   on_tap=open_sensor_detail("TEMPERATURE")),
        chart_card("H2S (ppm)", ft.Colors.YELLOW_400, h2s_chart,
                   create_maximize_handler("H2S (ppm)", ft.Colors.YELLOW_400, h2s_chart, h2s_georef),
                   [("H2S", ft.Colors.YELLOW_400, lambda e: show_band_config("H2S"))],
                   on_tap=open_sensor_detail("H2S")),
        chart_card("SO₂ (ppm)", ft.Colors.RED_400, so2_chart,
                   create_maximize_handler("SO₂ (ppm)", ft.Colors.RED_400, so2_chart, so2_georef),
                   [("SO₂", ft.Colors.RED_400, lambda e: show_band_config("SO2"))],
                   on_tap=open_sensor_detail("SO2")),
    ]

    detail_card = chart_card(
        "Vista ampliada", ft.Colors.PURPLE_400, detail_chart,
        create_maximize_handler("Vista ampliada", ft.Colors.PURPLE_400, detail_chart, co2_georef),
    )

    def on_dashboard_tab_change(e):
        dashboard_tab_index[0] = e.control.selected_index
        active_charts = (
            [co2_individual_chart, co2fine_chart, hum_chart, temp_chart, h2s_chart, so2_chart]
            if dashboard_tab_index[0] == 0 else [detail_chart]
        )
        for c in active_charts:
            refresh_secondary_chart(c)
        try:
            for c in active_charts:
                c.update()
        except Exception:
            schedule_ui_refresh()

    table_view = ft.ListView(expand=True, spacing=0)

    def refresh_table_view():
        """Rebuild the Excel-style table from the most recent telemetry."""
        table_view.controls = [
            ft.Row([build_data_table()], scroll=ft.ScrollMode.AUTO),
        ]

    refresh_table_view()

    grid_view = ft.Column([
        ft.Row([
            ft.Column([
                ft.Row([individual_cards[0], individual_cards[1]], spacing=10, expand=1),
                ft.Row([individual_cards[2], individual_cards[3]], spacing=10, expand=1),
                ft.Row([individual_cards[4], individual_cards[5]], spacing=10, expand=1),
            ], spacing=10, expand=1),
            scatter_container,
        ], spacing=10, expand=1),
    ], expand=True)
    individual_view = ft.Column([
        detail_heading,
        ft.Text("Haz clic en cualquier gráfica de la cuadrícula para mostrarla aquí.",
                size=11, color=theme["text_secondary"]),
        detail_card,
    ], spacing=8, expand=True)
    data_view = ft.Column([
        ft.Text("Datos de telemetría", size=16, weight="bold", color=theme["text_primary"]),
        ft.Text("Últimas 250 muestras recibidas. Desplázate horizontalmente para ver todas las columnas.",
                size=11, color=theme["text_secondary"]),
        table_view,
    ], spacing=8, expand=True)
    dashboard_tabs = ft.Tabs(
        length=3,
        selected_index=0,
        animation_duration=200,
        on_change=on_dashboard_tab_change,
        content=ft.Column([
            ft.TabBar(tabs=[
                ft.Tab(label="Cuadrícula", icon=ft.Icons.GRID_VIEW),
                ft.Tab(label="Detalle", icon=ft.Icons.SHOW_CHART),
                ft.Tab(label="Datos", icon=ft.Icons.TABLE_ROWS),
            ]),
            ft.TabBarView(controls=[grid_view, individual_view, data_view], expand=True),
        ], expand=True, spacing=0),
        expand=True,
    )

    # --- Build Page ---
    page.remove(loading_container)
    page.add(
        ft.Row([
            ft.Column([
                ft.Text("MiniHawk", size=24, weight="bold", color=theme["text_primary"]),
                ft.Row([
                    ft.Column([
                        ft.Text("GPS QUALITY", size=9, color=theme["label_color"]),
                        gps_badge,
                    ], spacing=2),
                    ft.Column([
                        ft.Text("POSITION", size=9, color=theme["label_color"]),
                        pos_badge,
                    ], spacing=2),
                ], spacing=8),
            ]),
            ft.Row([
                ft.Column([
                    ft.Row([
                        conn_dot,
                        sample_rate_val,
                    ], spacing=6, vertical_alignment=ft.CrossAxisAlignment.CENTER),
                ], spacing=0, alignment=ft.MainAxisAlignment.CENTER),
                ft.FilledButton("Export CSV", icon=ft.Icons.SAVE_ALT, on_click=download_csv),
                log_btn,
                offline_btn,
                theme_btn,
                settings_btn,
                ft.Image(
                    src="logo.png",
                    width=128,
                    height=128,
                    fit="contain",
                    border_radius=ft.BorderRadius.all(6),
                ),
            ], spacing=12),
        ], alignment=ft.MainAxisAlignment.SPACE_BETWEEN),

        dashboard_tabs,

        georef_panel,
    )

    sensor_states = {
        "CO2": {"charts": [co2_chart, co2_individual_chart], "pts": co2_pts, "georef": co2_georef, "x_key": "CO2", "color": ft.Colors.PURPLE_400},
        "CO2_FINE": {"charts": [co2_chart, co2fine_chart], "pts": co2fine_pts, "georef": co2fine_georef, "x_key": "CO2_FINE", "color": ft.Colors.CYAN_400},
        "HUMIDITY": {"charts": [climate_chart, hum_chart], "pts": hum_pts, "georef": hum_georef, "x_key": "HUM", "color": ft.Colors.GREEN_400},
        "TEMPERATURE": {"charts": [climate_chart, temp_chart], "pts": temp_pts, "georef": temp_georef, "x_key": "TEMP", "color": ft.Colors.ORANGE_400},
        "H2S": {"charts": [gas_chart, h2s_chart], "pts": h2s_pts, "georef": h2s_georef, "x_key": "H2S", "color": ft.Colors.YELLOW_400},
        "SO2": {"charts": [gas_chart, so2_chart], "pts": so2_pts, "georef": so2_georef, "x_key": "SO2", "color": ft.Colors.RED_400},
    }
    chart_sensor_names = {
        id(co2_chart): ["CO2", "CO2_FINE"],
        id(climate_chart): ["HUMIDITY", "TEMPERATURE"],
        id(gas_chart): ["H2S", "SO2"],
        id(co2_individual_chart): ["CO2"],
        id(co2fine_chart): ["CO2_FINE"],
        id(hum_chart): ["HUMIDITY"],
        id(temp_chart): ["TEMPERATURE"],
        id(h2s_chart): ["H2S"],
        id(so2_chart): ["SO2"],
        id(detail_chart): [detail_selection[0]],
    }

    def _rebuild_chart_series(chart):
        sensor_names = chart_sensor_names[id(chart)]
        all_points = [point for name in sensor_names for point in sensor_states[name]["pts"]]
        if not all_points:
            chart.data_series = [
                fch.LineChartData(
                    color=sensor_states[name]["color"], stroke_width=2, curved=True,
                    rounded_stroke_cap=True, points=[],
                )
                for name in sensor_names
            ]
            chart.min_x = 0
            chart.max_x = MAX_POINTS
            chart.min_y = -1
            chart.max_y = 1
            return
        ys = [p.y for p in all_points]
        y_min, y_max = min(ys), max(ys)
        margin = max((y_max - y_min) * 0.40, 3.0)
        lo  = y_min - margin
        hi  = y_max + margin
        mid = (lo + hi) / 2

        rebuilt_series = []
        for name in sensor_names:
            series_state = sensor_states[name]
            series_color = series_state["color"]
            series_points = series_state["pts"]
            if config.get("use_bands", {}).get(name, False) and series_points:
                latest = series_points[-1].y
                bands = config.get("bands", {}).get(name, [])
                band_color = next((b["color"] for b in bands if latest <= b["max"]), "white")
                series_color = COLOR_MAP.get(band_color, series_color)
            rebuilt_series.append(fch.LineChartData(
                color=series_color, stroke_width=2, curved=True,
                rounded_stroke_cap=True, points=list(series_points),
            ))
        chart.data_series = rebuilt_series
        max_x = max((p.x for p in all_points), default=MAX_POINTS)
        chart.min_x = max(0, max_x - MAX_POINTS)
        chart.max_x = max(MAX_POINTS, max_x)
        chart.min_y = lo
        chart.max_y = hi
        lbl_color = theme["label_color"]
        chart.left_axis.labels = [
            fch.ChartAxisLabel(value=lo,  label=ft.Text(f"{lo:.1f}",  size=9, color=lbl_color)),
            fch.ChartAxisLabel(value=mid, label=ft.Text(f"{mid:.1f}", size=9, color=lbl_color)),
            fch.ChartAxisLabel(value=hi,  label=ft.Text(f"{hi:.1f}",  size=9, color=lbl_color)),
        ]
        chart.horizontal_grid_lines.interval = max((hi - lo) / 4, 0.1)

    def refresh_secondary_chart(chart):
        """Keep the alternate view current when displayed."""
        _rebuild_chart_series(chart)

    # --- Push point helper: reconstruye series y marca gráfico sucio para render loop ---
    def push_point(sensor_name, val):
        state = sensor_states[sensor_name]
        pts = state["pts"]
        georef = state["georef"]
        xi = x_idx[state["x_key"]]
        georef.append({
            "val": val,
            "lat": gps_state["lat"],
            "lon": gps_state["lon"],
            "alt": gps_state["alt"],
        })
        if len(georef) > MAX_POINTS:
            georef.pop(0)

        tooltip_str = (
            f"{val:.2f} | {gps_state['lat']:.4f}° {gps_state['lon']:.4f}° | ↑{gps_state['alt']:.1f}m"
        )
        pts.append(fch.LineChartDataPoint(
            x=xi, y=val,
            show_tooltip=True,
            tooltip=tooltip_str,
        ))
        if len(pts) > MAX_POINTS:
            pts.pop(0)

        grid_chart = state["charts"][1]
        _rebuild_chart_series(grid_chart)
        dirty_charts.add(grid_chart)

        if detail_selection[0] == sensor_name:
            _rebuild_chart_series(detail_chart)
            dirty_charts.add(detail_chart)

        # Refresh maximized dialog for this chart if open
        active_chart = detail_chart if dashboard_tab_index[0] == 1 and detail_selection[0] == sensor_name else grid_chart
        dlg = line_chart_dlgs.get(active_chart) or line_chart_dlgs.get(grid_chart)
        if dlg and getattr(dlg, "open", False):
            try:
                live_series = list(active_chart.data_series or [])
                dlg_chart = fch.LineChart(
                    data_series=[
                        fch.LineChartData(
                            color=source.color,
                            stroke_width=3,
                            curved=True,
                            rounded_stroke_cap=True,
                            points=list(source.points),
                        )
                        for source in live_series
                    ],
                    expand=True,
                    min_x=active_chart.min_x,
                    max_x=active_chart.max_x,
                    min_y=active_chart.min_y,
                    max_y=active_chart.max_y,
                    animation=0,
                    on_event=active_chart.on_event,
                    horizontal_grid_lines=fch.ChartGridLines(
                        interval=active_chart.horizontal_grid_lines.interval,
                        color=theme["grid_color"], width=1,
                    ),
                    vertical_grid_lines=fch.ChartGridLines(
                        interval=active_chart.vertical_grid_lines.interval,
                        color=theme["grid_color"], width=1,
                    ),
                    left_axis=fch.ChartAxis(show_labels=True, label_size=36, labels=active_chart.left_axis.labels),
                    bottom_axis=fch.ChartAxis(show_labels=True, label_size=16, labels=active_chart.bottom_axis.labels),
                    border=ft.Border.all(1, theme["axis_color"]),
                )
                dlg.content.content = ft.Column([dlg_chart], expand=True)
                dlg.update()
            except Exception:
                pass

    # --- Decoupled UI 30 FPS Render Loop ---
    async def ui_render_loop():
        while True:
            try:
                await asyncio.sleep(0.033)
                if dirty_charts:
                    active_charts = (
                        [co2_individual_chart, co2fine_chart, hum_chart, temp_chart, h2s_chart, so2_chart]
                        if dashboard_tab_index[0] == 0 else [detail_chart]
                    )
                    to_update = [c for c in active_charts if c in dirty_charts]
                    dirty_charts.clear()
                    for c in to_update:
                        try:
                            c.update()
                        except Exception:
                            pass

                if dirty_gps[0]:
                    dirty_gps[0] = False
                    try:
                        gps_badge.update()
                        pos_badge.update()
                    except Exception:
                        pass

                if dirty_rate[0]:
                    dirty_rate[0] = False
                    try:
                        sample_rate_val.update()
                    except Exception:
                        pass

                if dirty_conn[0]:
                    dirty_conn[0] = False
                    try:
                        conn_dot.update()
                    except Exception:
                        pass

                if dirty_table[0]:
                    dirty_table[0] = False
                    refresh_table_view()
                    try:
                        table_view.update()
                    except Exception:
                        pass
            except asyncio.CancelledError:
                break
            except Exception:
                pass

    # --- Render page first, then start MAVLink in background ---
    async def mavlink_task():
        add_log_message("Connecting to MAVLink...")
        asyncio.create_task(ui_render_loop())

        RECONNECT_SILENCE = 30.0   # seconds before full socket rebind
        INITIAL_BACKOFF = 2.0      # first reconnect pause
        MAX_BACKOFF = 10.0         # max reconnect pause
        backoff = INITIAL_BACKOFF

        while True:
            master = None
            bind_host = config.get("mavlink_bind_host", "0.0.0.0")
            port = config.get("mavlink_port", "14550")
            addr = f"udpin:{bind_host}:{port}"
            try:
                master = await asyncio.get_event_loop().run_in_executor(
                    None, lambda: mavutil.mavlink_connection(addr)
                )
                add_log_message(f"MAVLink: listening on {addr}, waiting for data... (backoff={backoff:.0f}s)")
            except Exception as e:
                add_log_message(f"UDP Error: {e}")
                await asyncio.sleep(backoff)
                backoff = min(backoff * 1.5, MAX_BACKOFF)
                continue

            last_msg_time = asyncio.get_event_loop().time()
            connection_lost_logged = False
            ever_received = False

            def _update_conn_dot_inner(state: int):
                if state == 1:
                    conn_dot.bgcolor = ft.Colors.GREEN
                    conn_dot.tooltip = "MAVLink active"
                elif state == -1:
                    conn_dot.bgcolor = ft.Colors.RED
                    conn_dot.tooltip = "MAVLink lost"
                else:
                    conn_dot.bgcolor = ft.Colors.YELLOW
                    conn_dot.tooltip = "Waiting for MAVLink..."
                dirty_conn[0] = True

            def _update_sample_rate():
                now = time.time()
                dt = now - _rate_counts["_last_reset"]
                if dt >= 1.0:
                    hz = (_rate_counts["CO2"] + _rate_counts["HUMIDITY"] + _rate_counts["TEMP"] + _rate_counts["H2S"] + _rate_counts["SO2"] + _rate_counts["CO2_FINE"]) / dt
                    sample_rate_val.value = f"{hz:.1f} Hz"
                    dirty_rate[0] = True
                    _rate_counts["_last_reset"] = now
                    _rate_counts["CO2"] = 0
                    _rate_counts["HUMIDITY"] = 0
                    _rate_counts["TEMP"] = 0
                    _rate_counts["H2S"] = 0
                    _rate_counts["SO2"] = 0
                    _rate_counts["CO2_FINE"] = 0

            _update_conn_dot_inner(0)

            while True:
                msg_count = 0
                while True:
                    msg = master.recv_match(blocking=False)
                    if not msg:
                        break
                    msg_count += 1
                    last_msg_time = asyncio.get_event_loop().time()
                    connection_lost_logged = False
                    if not ever_received:
                        ever_received = True
                        add_log_message("MAVLink stream active ✓", theme["text_green"])
                        _update_conn_dot_inner(1)
                        backoff = INITIAL_BACKOFF
                    m_type = msg.get_type()

                    if m_type == "GPS_RAW_INT":
                        gps_state["fix_type"] = msg.fix_type
                        gps_state["satellites"] = msg.satellites_visible
                        gps_state["eph_cm"] = msg.eph
                        gps_badge.content.value = format_gps_badge(msg.fix_type, msg.satellites_visible, msg.eph)
                        gps_badge.bgcolor = get_gps_quality_color(msg.fix_type, msg.satellites_visible, msg.eph, theme)
                        dirty_gps[0] = True
                    elif m_type == "GLOBAL_POSITION_INT":
                        gps_state["lat"] = msg.lat / 1e7
                        gps_state["lon"] = msg.lon / 1e7
                        gps_state["alt"] = msg.relative_alt / 1000.0
                        pos_badge.content.value = (
                            f"{gps_state['lat']:.4f}\u00b0  "
                            f"{gps_state['lon']:.4f}\u00b0  "
                            f"\u2191{gps_state['alt']:.1f}m"
                        )
                        dirty_gps[0] = True

                    elif m_type == "NAMED_VALUE_FLOAT":
                        name = msg.name.strip('\x00').strip().upper()
                        val  = msg.value
                        telemetry_history.append({
                            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                            "name": name, "value": val,
                            "lat": gps_state["lat"],
                            "lon": gps_state["lon"],
                            "alt": gps_state["alt"],
                        })
                        dirty_table[0] = True

                        if name == "CO2":
                            push_point("CO2", val)
                            x_idx["CO2"] += 1
                            gps_co2_data.append((gps_state["lat"], gps_state["lon"], val, gps_state["alt"]))
                            if len(gps_co2_data) > MAX_POINTS:
                                gps_co2_data.pop(0)
                            now = time.time()
                            if now - _last_scatter_time[0] >= SCATTER_MIN_INTERVAL:
                                _last_scatter_time[0] = now
                                if _scatter_render_task[0] is None or _scatter_render_task[0].done():
                                    _scatter_render_task[0] = asyncio.create_task(_update_scatter_plot_async())
                        elif name == "HUMIDITY":
                            push_point("HUMIDITY", val)
                            x_idx["HUM"] += 1
                        elif name == "TEMP":
                            push_point("TEMPERATURE", val)
                            x_idx["TEMP"] += 1
                        elif name == "H2S":
                            push_point("H2S", val)
                            x_idx["H2S"] += 1
                        elif name == "SO2":
                            push_point("SO2", val)
                            x_idx["SO2"] += 1
                        elif name == "CO2_FINE":
                            push_point("CO2_FINE", val)
                            x_idx["CO2_FINE"] += 1

                        _rate_counts[name] = _rate_counts.get(name, 0) + 1
                        _update_sample_rate()

                        add_log_message(
                            f"{name}: {val:.4f}  |  "
                            f"{gps_state['lat']:.4f}° {gps_state['lon']:.4f}°  "
                            f"↑{gps_state['alt']:.1f}m"
                        )

                    if msg_count >= 50:
                        break

                if msg_count == 0:
                    await asyncio.sleep(0.005)
                else:
                    await asyncio.sleep(0.001)

                now = asyncio.get_event_loop().time()
                elapsed = now - last_msg_time
                if elapsed > 10 and not connection_lost_logged:
                    if ever_received:
                        add_log_message("MAVLink: connection lost, waiting...")
                        _update_conn_dot_inner(-1)
                    else:
                        add_log_message(
                            "MAVLink: no autopilot data on 14550. "
                            "(Mission Planner mirror only forwards when drone is linked)"
                        )
                    connection_lost_logged = True

                if elapsed > RECONNECT_SILENCE:
                    add_log_message(f"MAVLink: reconnecting in {backoff:.0f}s...")
                    master.close()
                    await asyncio.sleep(backoff)
                    break  # break inner loop to rebind in outer loop

                # Handle settings restart request
                if restart_event.is_set():
                    restart_event.clear()
                    add_log_message("MAVLink: restarting with new address...")
                    master.close()
                    break  # rebind in outer loop with new config

            # Outer loop: rebind with capped exponential backoff
            backoff = min(backoff * 1.5, MAX_BACKOFF)

    asyncio.get_event_loop().create_task(mavlink_task())


if __name__ == "__main__":
    ft.run(main)
