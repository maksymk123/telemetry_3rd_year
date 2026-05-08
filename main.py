import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import re
import time
from dataclasses import dataclass, field
from tkintermapview import TkinterMapView
from PIL import Image, ImageDraw, ImageTk

# Expected payload example (from LoRa RX):
# ID=DRONE1,LAT=53.464635,LON=-2.233527,ALT=47.9,HDOP=0.7,SATS=12,SPEED=3.5
#
# ID is optional; if missing, the app treats it as "DEFAULT".

_FLOAT = r"[+-]?\d+(?:\.\d+)?"


# RSSI: -85.00 dBm, SNR: 11.00 dB

def parse_kv_packet(line: str) -> dict[str, str]:
    """
    Parse packets like: ID=DRONE1,LAT=...,LON=...,TEMP=...
    Returns keys uppercased; values are raw strings (conversion happens later).
    """
    out: dict[str, str] = {}
    for part in line.split(","):
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        k = k.strip().upper()
        v = v.strip()
        if k:
            out[k] = v
    return out


RSSI_REGEX = re.compile(
    rf"RSSI:\s*({_FLOAT})\s*dBm,\s*SNR:\s*({_FLOAT})\s*dB",
    re.IGNORECASE,
)


@dataclass
class DroneState:
    drone_id: str
    has_fix: bool = False
    lat: float | None = None
    lon: float | None = None
    alt_m: float | None = None
    hdop: float | None = None
    sats: int | None = None
    speed_kmh: float | None = None

    # placeholders for future sensors
    temp_c: float | None = None
    pressure_hpa: float | None = None
    humidity_pct: float | None = None
    battery_pct: float | None = None
    battery_volt: float | None = None  # INA226: pack voltage (V)
    battery_amp: float | None = None  # INA226: current draw (A)
    battery_watt: float | None = None  # INA226: power (W)

    baro_alt_m: float | None = None
    rel_alt_m: float | None = None
    vspd_mps: float | None = None

    rssi_dbm: float | None = None
    snr_db: float | None = None
    last_seen_ts: float | None = None

    marker = None
    path = None
    path_points: list[tuple[float, float]] = field(default_factory=list)


class GPSMapApp:
    def __init__(self, root):
        self.root = root
        self.root.title("LoRa GPS Map Receiver (Multi-Drone)")

        self.serial_port = None
        self.running = False

        # Multi-drone state
        self.drones: dict[str, DroneState] = {}
        self.selected_drone_id: str | None = None
        self._last_rx_drone_id: str | None = None  # used to attach RSSI/SNR to the last GPS payload

        # Path colors for multiple drones
        self.path_palette = [
            "red", "blue", "green", "orange", "purple",
            "cyan", "magenta", "gold", "white", "pink"
        ]
        self.drone_colors: dict[str, str] = {}
        self.drone_icon_cache: dict[str, ImageTk.PhotoImage] = {}
        self.legend_items: dict[str, tuple[tk.Frame, tk.Label]] = {}

        # ---- TOP BAR ----
        top_frame = ttk.Frame(root)
        top_frame.pack(fill="x", padx=5, pady=5)

        ttk.Label(top_frame, text="Serial Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_menu = ttk.Combobox(top_frame, textvariable=self.port_var, width=28, state="readonly")
        self.port_menu.pack(side="left", padx=5)

        ttk.Button(top_frame, text="Refresh", command=self.refresh_ports).pack(side="left", padx=5)
        ttk.Button(top_frame, text="Connect", command=self.toggle_connection).pack(side="left", padx=5)

        ttk.Separator(top_frame, orient="vertical").pack(side="left", fill="y", padx=8)

        ttk.Label(top_frame, text="Drone:").pack(side="left")
        self.drone_var = tk.StringVar()
        self.drone_menu = ttk.Combobox(top_frame, textvariable=self.drone_var, width=18, state="readonly")
        self.drone_menu.pack(side="left", padx=5)
        self.drone_menu.bind("<<ComboboxSelected>>", lambda _e: self.on_select_drone())

        ttk.Button(top_frame, text="Focus on selected", command=self.focus_on_selected).pack(side="left", padx=5)

        self.follow_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top_frame, text="Follow selected", variable=self.follow_var).pack(side="left", padx=8)

        # Status label (GPS fix info)
        self.status_label = ttk.Label(root, text="Disconnected", foreground="red")
        self.status_label.pack(pady=3)

        # ---- LEGEND ----
        self.legend_frame = ttk.LabelFrame(root, text="Drone legend")
        self.legend_frame.pack(fill="x", padx=8, pady=(0, 6))

        # ---- INFO BOX ----
        info_frame = ttk.LabelFrame(root, text="Telemetry (selected drone)")
        info_frame.pack(fill="x", padx=8, pady=6)

        self.speed_var = tk.StringVar(value="—")
        self.temp_var = tk.StringVar(value="—")
        self.pressure_var = tk.StringVar(value="—")
        self.humidity_var = tk.StringVar(value="—")
        self.baro_alt_var = tk.StringVar(value="—")
        self.rel_alt_var = tk.StringVar(value="—")
        self.vspd_var = tk.StringVar(value="—")
        self.battery_var = tk.StringVar(value="—")
        self.battery_volt_var = tk.StringVar(value="—")
        self.battery_amp_var = tk.StringVar(value="—")
        self.battery_watt_var = tk.StringVar(value="—")
        self.rssi_var = tk.StringVar(value="—")
        self.snr_var = tk.StringVar(value="—")

        ttk.Label(info_frame, text="Speed (km/h):").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(info_frame, textvariable=self.speed_var).grid(row=0, column=1, sticky="w")

        ttk.Label(info_frame, text="Temperature (°C):").grid(row=1, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(info_frame, textvariable=self.temp_var).grid(row=1, column=1, sticky="w")

        ttk.Label(info_frame, text="Pressure (hPa):").grid(row=2, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(info_frame, textvariable=self.pressure_var).grid(row=2, column=1, sticky="w")

        ttk.Label(info_frame, text="Humidity (%):").grid(row=3, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(info_frame, textvariable=self.humidity_var).grid(row=3, column=1, sticky="w")

        ttk.Label(info_frame, text="Baro Alt (m):").grid(row=4, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(info_frame, textvariable=self.baro_alt_var).grid(row=4, column=1, sticky="w")
        ttk.Label(info_frame, text="Rel Alt (m):").grid(row=5, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(info_frame, textvariable=self.rel_alt_var).grid(row=5, column=1, sticky="w")

        ttk.Label(info_frame, text="VSpeed (m/s):").grid(row=6, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(info_frame, textvariable=self.vspd_var).grid(row=6, column=1, sticky="w")

        ttk.Label(info_frame, text="Battery (%):").grid(row=7, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(info_frame, textvariable=self.battery_var).grid(row=7, column=1, sticky="w")

        ttk.Label(info_frame, text="Battery (V):").grid(row=8, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(info_frame, textvariable=self.battery_volt_var).grid(row=8, column=1, sticky="w")

        ttk.Label(info_frame, text="Current (A):").grid(row=9, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(info_frame, textvariable=self.battery_amp_var).grid(row=9, column=1, sticky="w")

        ttk.Label(info_frame, text="Power (W):").grid(row=10, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(info_frame, textvariable=self.battery_watt_var).grid(row=10, column=1, sticky="w")

        ttk.Label(info_frame, text="RSSI (dBm):").grid(row=11, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(info_frame, textvariable=self.rssi_var).grid(row=11, column=1, sticky="w")

        ttk.Label(info_frame, text="SNR (dB):").grid(row=12, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(info_frame, textvariable=self.snr_var).grid(row=12, column=1, sticky="w")

        for c in range(2):
            info_frame.grid_columnconfigure(c, weight=1)

        # ---- MAP ----
        self.map_widget = TkinterMapView(root, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_zoom(4)
        self.map_widget.set_position(20, 0)

        self.refresh_ports()

    # -------- PORTS --------
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        lst = [p.device for p in ports]
        self.port_menu["values"] = lst
        if lst and not self.port_var.get():
            self.port_var.set(lst[0])

    def toggle_connection(self):
        if not self.running:
            self.start_serial()
        else:
            self.stop_serial()

    def start_serial(self):
        try:
            # Your receiver sketch prints at 115200; keep this at 115200.
            self.serial_port = serial.Serial(self.port_var.get(), 115200, timeout=1)
            self.running = True
            self.status_label.config(text=f"Connected to {self.port_var.get()}", foreground="green")
            threading.Thread(target=self.read_serial, daemon=True).start()
        except Exception as e:
            self.status_label.config(text=f"Error: {e}", foreground="red")

    def stop_serial(self):
        self.running = False
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None

        # Clear map objects
        for d in self.drones.values():
            try:
                if d.marker:
                    d.marker.delete()
            except Exception:
                pass
            try:
                if d.path:
                    d.path.delete()
            except Exception:
                pass

        self.drones.clear()
        self.selected_drone_id = None
        self._last_rx_drone_id = None
        self.drone_menu["values"] = []
        self.drone_var.set("")

        self.clear_info_box()
        self.status_label.config(text="Disconnected", foreground="red")

    # -------- SERIAL LOOP --------
    def read_serial(self):
        while self.running:
            try:
                line = self.serial_port.readline().decode(errors="ignore").strip()
                if not line:
                    continue

                # Debug raw line (optional)
                # print("[SERIAL RAW]", line)
                # Telemetry packets are KEY=VALUE comma-separated
                if "=" in line:
                    kv = parse_kv_packet(line)
                    if "LAT" in kv and "LON" in kv:
                        drone_id = (kv.get("ID") or "DEFAULT").strip()
                        self._last_rx_drone_id = drone_id

                        def f(key: str):
                            try:
                                v = kv[key]
                                if v in ("NA", "NaN", "na"):
                                    return None
                                return float(v)
                            except Exception:
                                return None

                        def i(key: str):
                            try:
                                v = kv[key]
                                if v in ("NA", "NaN", "na"):
                                    return None
                                return int(float(v))
                            except Exception:
                                return None

                        has_fix = kv.get("FIX", "0") == "1"
                        lat = f("LAT")
                        lon = f("LON")
                        # Always update — even without a fix, sensor data is useful.
                        # lat/lon will be None when FIX=0; the map marker is skipped.
                        self.update_drone_packet(
                            drone_id=drone_id,
                            lat=lat,
                            lon=lon,
                            has_fix=has_fix,
                            alt=f("ALT") if "ALT" in kv else None,
                            hdop=f("HDOP") if "HDOP" in kv else None,
                            sats=i("SATS") if "SATS" in kv else None,
                            speed=f("SPEED") if "SPEED" in kv else None,
                            temp_c=f("TEMP") if "TEMP" in kv else None,
                            humidity_pct=f("HUM") if "HUM" in kv else None,
                            pressure_hpa=f("PRES") if "PRES" in kv else None,
                            baro_alt_m=f("BALT") if "BALT" in kv and kv.get("BALT") not in ("NA", "NaN") else None,
                            rel_alt_m=f("RALT") if "RALT" in kv and kv.get("RALT") not in ("NA", "NaN") else None,
                            vspd_mps=f("VSPD") if "VSPD" in kv and kv.get("VSPD") not in ("NA", "NaN") else None,
                            battery_pct=f("BATT") if "BATT" in kv and kv.get("BATT") not in ("NA", "NaN") else None,
                            battery_volt=f("BVOLT") if "BVOLT" in kv and kv.get("BVOLT") not in ("NA", "NaN") else None,
                            battery_amp=f("BAMP") if "BAMP" in kv and kv.get("BAMP") not in ("NA", "NaN") else None,
                            battery_watt=f("BWATT") if "BWATT" in kv and kv.get("BWATT") not in ("NA", "NaN") else None,
                        )
                        continue
                rssi_match = RSSI_REGEX.search(line)
                if rssi_match and self._last_rx_drone_id:
                    rssi = float(rssi_match.group(1))
                    snr = float(rssi_match.group(2))
                    self.update_drone_link(self._last_rx_drone_id, rssi, snr)
                    continue

            except Exception as e:
                print("[ERROR]", e)

    # -------- DRONE STATE / UI --------
    def _ensure_drone(self, drone_id: str) -> DroneState:
        d = self.drones.get(drone_id)
        if d is None:
            d = DroneState(drone_id=drone_id)
            self.drones[drone_id] = d
            self.root.after(0, self._refresh_drone_list_ui)
        return d

    def _refresh_drone_list_ui(self):
        ids = sorted(self.drones.keys())
        self.drone_menu["values"] = ids
        self.refresh_legend()
        if self.selected_drone_id is None and ids:
            self.selected_drone_id = ids[0]
            self.drone_var.set(self.selected_drone_id)
            self.on_select_drone()

    def on_select_drone(self):
        self.selected_drone_id = self.drone_var.get() or None
        self.render_paths()
        self.update_info_box()

    def get_drone_color(self, drone_id: str) -> str:
        color = self.drone_colors.get(drone_id)
        if color is None:
            color = self.path_palette[len(self.drone_colors) % len(self.path_palette)]
            self.drone_colors[drone_id] = color
        return color

    def _hex_to_rgb(self, color_name: str) -> tuple[int, int, int]:
        """Convert a Tk color name into an RGB tuple."""
        r16, g16, b16 = self.root.winfo_rgb(color_name)
        return (r16 // 256, g16 // 256, b16 // 256)

    def _make_drone_icon(self, color_name: str) -> ImageTk.PhotoImage:
        """Create a small drone-shaped icon tinted with the drone color."""
        rgb = self._hex_to_rgb(color_name)
        img = Image.new("RGBA", (34, 34), (0, 0, 0, 0))
        draw = ImageDraw.Draw(img)

        # Arms
        draw.line((9, 17, 25, 17), fill=rgb + (255,), width=3)
        draw.line((17, 9, 17, 25), fill=rgb + (255,), width=3)

        # Body
        draw.rounded_rectangle((12, 12, 22, 22), radius=4, fill=(40, 40, 40, 255), outline=rgb + (255,), width=2)

        # Rotors
        for cx, cy in ((7, 7), (27, 7), (7, 27), (27, 27)):
            draw.ellipse((cx - 4, cy - 4, cx + 4, cy + 4), fill=rgb + (220,), outline=(20, 20, 20, 255), width=1)

        # Nose dot
        draw.ellipse((15, 8, 19, 12), fill=(255, 255, 255, 240))

        return ImageTk.PhotoImage(img)

    def _get_drone_icon(self, drone_id: str) -> ImageTk.PhotoImage:
        color = self.get_drone_color(drone_id)
        icon = self.drone_icon_cache.get(drone_id)
        if icon is None:
            icon = self._make_drone_icon(color)
            self.drone_icon_cache[drone_id] = icon
        return icon

    def refresh_legend(self):
        """Update the legend so every drone shows its assigned color."""
        current_ids = set(self.drones.keys())

        # Remove legend rows for drones that no longer exist
        for drone_id in list(self.legend_items.keys()):
            if drone_id not in current_ids:
                frame, _label = self.legend_items.pop(drone_id)
                frame.destroy()

        # Add / update rows
        sorted_ids = sorted(current_ids)
        for row, drone_id in enumerate(sorted_ids):
            color = self.get_drone_color(drone_id)
            if drone_id not in self.legend_items:
                row_frame = ttk.Frame(self.legend_frame)
                swatch = tk.Label(row_frame, width=2, background=color, relief="solid", borderwidth=1)
                swatch.pack(side="left", padx=(0, 6))
                label = ttk.Label(row_frame, text=drone_id)
                label.pack(side="left")
                row_frame.grid(row=row, column=0, sticky="w", padx=6, pady=2)
                self.legend_items[drone_id] = (row_frame, label)
            else:
                row_frame, label = self.legend_items[drone_id]
                row_frame.grid(row=row, column=0, sticky="w", padx=6, pady=2)
                label.configure(text=drone_id)

    def focus_on_selected(self):
        d = self.get_selected_drone()
        if d and d.lat is not None and d.lon is not None:
            self.map_widget.set_position(d.lat, d.lon)

    def get_selected_drone(self) -> DroneState | None:
        if self.selected_drone_id is None:
            return None
        return self.drones.get(self.selected_drone_id)

    def clear_info_box(self):
        self.speed_var.set("—")
        self.temp_var.set("—")
        self.pressure_var.set("—")
        self.humidity_var.set("—")
        self.baro_alt_var.set("—")
        self.rel_alt_var.set("—")
        self.vspd_var.set("—")
        self.battery_var.set("—")
        self.battery_volt_var.set("—")
        self.battery_amp_var.set("—")
        self.battery_watt_var.set("—")
        self.rssi_var.set("—")
        self.snr_var.set("—")

    def update_info_box(self):
        d = self.get_selected_drone()
        if not d:
            self.clear_info_box()
            return

        self.speed_var.set(f"{d.speed_kmh:.1f}" if d.speed_kmh is not None else "—")
        self.temp_var.set(f"{d.temp_c:.1f}" if d.temp_c is not None else "—")
        self.pressure_var.set(f"{d.pressure_hpa:.1f}" if d.pressure_hpa is not None else "—")
        self.humidity_var.set(f"{d.humidity_pct:.1f}" if d.humidity_pct is not None else "—")
        self.baro_alt_var.set(f"{d.baro_alt_m:.1f}" if d.baro_alt_m is not None else "—")
        self.rel_alt_var.set(f"{d.rel_alt_m:.1f}" if d.rel_alt_m is not None else "—")
        self.vspd_var.set(f"{d.vspd_mps:.2f}" if d.vspd_mps is not None else "—")
        self.battery_var.set(f"{d.battery_pct:.0f}%" if d.battery_pct is not None else "—")
        self.battery_volt_var.set(f"{d.battery_volt:.2f} V" if d.battery_volt is not None else "—")
        self.battery_amp_var.set(f"{d.battery_amp:.2f} A" if d.battery_amp is not None else "—")
        self.battery_watt_var.set(f"{d.battery_watt:.1f} W" if d.battery_watt is not None else "—")
        self.rssi_var.set(f"{d.rssi_dbm:.1f}" if d.rssi_dbm is not None else "—")
        self.snr_var.set(f"{d.snr_db:.1f}" if d.snr_db is not None else "—")

        if d.last_seen_ts:
            age = time.time() - d.last_seen_ts
            if not d.has_fix:
                self.status_label.config(
                    text=f"[{d.drone_id}]  ⚠ NO GPS FIX — SATS={d.sats or 0}  •  {age:.1f}s ago  (sensor data live)",
                    foreground="orange",
                )
            elif d.lat is not None and d.lon is not None and d.alt_m is not None:
                self.status_label.config(
                    text=f"[{d.drone_id}] LAT={d.lat:.6f}, LON={d.lon:.6f}, ALT={d.alt_m:.1f} m  •  {age:.1f}s ago",
                    foreground="green",
                )

    # -------- MAP UPDATE --------

    def update_drone_packet(
            self,
            drone_id: str,
            lat: float | None,
            lon: float | None,
            has_fix: bool = False,
            alt: float | None = None,
            hdop: float | None = None,
            sats: int | None = None,
            speed: float | None = None,
            temp_c: float | None = None,
            humidity_pct: float | None = None,
            pressure_hpa: float | None = None,
            baro_alt_m: float | None = None,
            rel_alt_m: float | None = None,
            vspd_mps: float | None = None,
            battery_pct: float | None = None,
            battery_volt: float | None = None,
            battery_amp: float | None = None,
            battery_watt: float | None = None,
    ):
        """Thread-safe: schedules a GUI-thread update for all telemetry fields."""

        def gui_update():
            d = self._ensure_drone(drone_id)

            # Core telemetry
            d.has_fix = has_fix
            if lat is not None: d.lat = lat
            if lon is not None: d.lon = lon
            if alt is not None: d.alt_m = alt
            if hdop is not None: d.hdop = hdop
            if sats is not None: d.sats = sats
            if speed is not None: d.speed_kmh = speed

            # BME / extras
            if temp_c is not None: d.temp_c = temp_c
            if humidity_pct is not None: d.humidity_pct = humidity_pct
            if pressure_hpa is not None: d.pressure_hpa = pressure_hpa
            if baro_alt_m is not None: d.baro_alt_m = baro_alt_m
            if rel_alt_m is not None: d.rel_alt_m = rel_alt_m
            if vspd_mps is not None: d.vspd_mps = vspd_mps
            if battery_pct is not None: d.battery_pct = battery_pct
            if battery_volt is not None: d.battery_volt = battery_volt
            if battery_amp is not None: d.battery_amp = battery_amp
            if battery_watt is not None: d.battery_watt = battery_watt

            d.last_seen_ts = time.time()

            # Only place/move map marker when we have a real GPS fix
            if has_fix and lat is not None and lon is not None:
                if d.marker is None:
                    color = self.get_drone_color(drone_id)
                    d.marker = self.map_widget.set_marker(
                        lat, lon,
                        text=drone_id,
                        icon=self._get_drone_icon(drone_id),
                        icon_anchor="center",
                        marker_color_circle=color,
                        marker_color_outside=color,
                    )
                else:
                    d.marker.set_position(lat, lon)

                d.path_points.append((lat, lon))

                if self.follow_var.get() and self.selected_drone_id == drone_id:
                    self.map_widget.set_position(lat, lon)

            # If nothing selected yet, select first drone we see
            if self.selected_drone_id is None:
                self.selected_drone_id = drone_id
                self.drone_var.set(drone_id)
                self._refresh_drone_list_ui()

            # Update selected-drone panel if this is the selected one
            if self.selected_drone_id == drone_id:
                self.update_info_box()
                self._draw_selected_path()

        self.root.after(0, gui_update)

    def update_drone_link(self, drone_id: str, rssi: float, snr: float):
        def gui_update():
            d = self._ensure_drone(drone_id)
            d.rssi_dbm = rssi
            d.snr_db = snr
            if self.selected_drone_id == drone_id:
                self.update_info_box()

        self.root.after(0, gui_update)

    def _draw_selected_path(self):
        self.render_paths()

    def render_paths(self):
        """Show and update the path for every drone with at least 2 points."""
        for did, d in self.drones.items():
            should_show = len(d.path_points) >= 2
            if should_show:
                color = self.get_drone_color(did)
                width = 5 if did == self.selected_drone_id else 3
                if d.path is None:
                    d.path = self.map_widget.set_path(d.path_points, color=color, width=width)
                else:
                    try:
                        # Rebuild path so all drones stay visually updated
                        d.path.delete()
                    except Exception:
                        pass
                    d.path = self.map_widget.set_path(d.path_points, color=color, width=width)
            else:
                if d.path is not None:
                    try:
                        d.path.delete()
                    except Exception:
                        pass
                    d.path = None


# ---- RUN ----
if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("980x780")
    app = GPSMapApp(root)
    root.mainloop()
