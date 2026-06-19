#!/usr/bin/env python3

import argparse
import csv
import html
import json
import math
from collections import Counter
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
from pyulog import ULog


DEFAULT_LOG_ROOT = Path("build/px4_sitl_default/rootfs/log")
DEFAULT_OUTPUT = Path("/tmp/ftc_indi_log_report.html")
DEFAULT_WIND_WORLD = Path("Tools/simulation/gz/worlds/iris_ftc_default.sdf")
DEFAULT_AERO_DEBUG = Path("/tmp/iris_ftc_aero_debug.csv")


TOPICS = (
    "vehicle_local_position",
    "vehicle_local_position_setpoint",
    "vehicle_thrust_setpoint",
    "vehicle_torque_setpoint",
    "vehicle_rates_setpoint",
    "vehicle_angular_velocity",
    "vehicle_acceleration",
    "vehicle_attitude",
    "vehicle_attitude_setpoint",
    "actuator_motors",
    "control_allocator_ftc_debug",
    "sensor_combined",
    "vehicle_ftc_physical_setpoint",
)


PARAMS = (
    "SYS_AUTOSTART",
    "MPC_THR_HOVER",
    "MC_FTC_NX",
    "MC_FTC_NY",
    "MC_FTC_NZ",
    "MC_FTC_KX",
    "MC_FTC_KY",
    "CA_FTC_MOT",
    "CA_FTC_LOE",
    "CA_FTC_KF",
    "CA_FTC_OMAX",
    "CA_FTC_ERPMAX",
    "CA_FTC_INDI_K1",
    "CA_FTC_INDI_K2",
    "CA_FTC_INDI_K3",
    "CA_FTC_INDI_ILIM",
    "CA_FTC_INDI_FC",
    "CA_FTC_INDI_SDL",
    "CA_FTC_INDI_M",
    "IMU_INTEG_RATE",
    "IMU_GYRO_RATEMAX",
)

PARAM_CHANGE_FILTER = set(PARAMS) | {
    "CA_FTC_STATE",
    "CA_FTC_EN",
    "CA_FTC_TYPE",
    "MC_ROLLRATE_MAX",
    "MC_PITCHRATE_MAX",
}


COLORS = (
    "#2563eb",
    "#dc2626",
    "#059669",
    "#f97316",
    "#7c3aed",
    "#111827",
    "#0891b2",
    "#be123c",
    "#65a30d",
    "#9333ea",
    "#0f766e",
    "#a16207",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a focused FTC INDI PX4 ULog report for position, PA+NDI, and INDI allocation debugging."
    )
    parser.add_argument("ulog", nargs="?", help="ULog path. If omitted, the newest SITL log is used.")
    parser.add_argument("-o", "--output", default=str(DEFAULT_OUTPUT), help="HTML output path.")
    parser.add_argument("--max-points", type=int, default=1800, help="Maximum points per trace after downsampling.")
    parser.add_argument("--event-padding", type=float, default=8.0, help="Seconds before/after FTC active interval to plot.")
    parser.add_argument("--full", action="store_true", help="Plot the full log instead of FTC event +/- padding.")
    parser.add_argument(
        "--wind-events",
        default="",
        help="Manual wind schedule: 'time:x:y:z[:enabled],...'. Example: '0:5:0:0,30:0:8:0,60:0:12:0'.",
    )
    parser.add_argument(
        "--wind-events-file",
        default="",
        help="CSV/JSON wind schedule. CSV columns: time_s,x,y,z[,enabled]. JSON: list of event objects.",
    )
    parser.add_argument(
        "--wind-world",
        default=str(DEFAULT_WIND_WORLD),
        help="World SDF used to infer the initial wind when no explicit t=0 wind event is present.",
    )
    parser.add_argument(
        "--aero-debug-file",
        default=str(DEFAULT_AERO_DEBUG),
        help="CSV emitted by IrisFtcAerodynamicsSystemPlugin.",
    )
    return parser.parse_args()


def newest_ulog() -> Path:
    candidates = sorted(DEFAULT_LOG_ROOT.glob("*/*.ulg"), key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"No .ulg files found under {DEFAULT_LOG_ROOT}")
    return candidates[-1]


def dataset_map(ulog: ULog) -> dict[str, dict[str, np.ndarray]]:
    result: dict[str, dict[str, np.ndarray]] = {}
    for data in ulog.data_list:
        if data.name in TOPICS and data.name not in result:
            result[data.name] = data.data
    return result


def time_s(data: dict[str, np.ndarray], t0_us: int) -> np.ndarray:
    return (np.asarray(data["timestamp"], dtype=np.float64) - float(t0_us)) * 1e-6


def field(data: dict[str, np.ndarray] | None, name: str) -> np.ndarray | None:
    if data is None or name not in data:
        return None
    return np.asarray(data[name], dtype=np.float64)


def find_event(debug: dict[str, np.ndarray] | None, t0_us: int) -> tuple[float | None, float | None]:
    if debug is None or "ftc_active" not in debug:
        return None, None

    t = time_s(debug, t0_us)
    active = np.asarray(debug["ftc_active"], dtype=np.int32) > 0

    if not np.any(active):
        return None, None

    idx = np.flatnonzero(active)
    return float(t[idx[0]]), float(t[idx[-1]])


def downsample(x: np.ndarray, y: np.ndarray, max_points: int) -> tuple[list[float], list[float]]:
    mask = np.isfinite(x) & np.isfinite(y)
    x = x[mask]
    y = y[mask]

    if len(x) == 0:
        return [], []

    if max_points > 0 and len(x) > max_points:
        indices = np.linspace(0, len(x) - 1, max_points).astype(np.int64)
        x = x[indices]
        y = y[indices]

    return np.round(x, 5).tolist(), np.round(y, 6).tolist()


def clip_window(x: np.ndarray, y: np.ndarray, start: float | None, end: float | None) -> tuple[np.ndarray, np.ndarray]:
    if start is None or end is None:
        return x, y
    mask = (x >= start) & (x <= end)
    return x[mask], y[mask]


def make_trace(
    data: dict[str, np.ndarray] | None,
    field_name: str,
    label: str,
    t0_us: int,
    start: float | None,
    end: float | None,
    max_points: int,
    scale: float = 1.0,
) -> dict[str, Any] | None:
    y = field(data, field_name)
    if y is None:
        return None

    x = time_s(data, t0_us)
    x, y = clip_window(x, y * scale, start, end)
    xp, yp = downsample(x, y, max_points)

    if not xp:
        return None

    return {"x": xp, "y": yp, "label": label}


def panel(title: str, ylabel: str, traces: list[dict[str, Any] | None]) -> dict[str, Any] | None:
    clean = [trace for trace in traces if trace is not None]
    if not clean:
        return None
    return {"title": title, "ylabel": ylabel, "traces": clean}


def bool_from_value(value: Any, default: bool = True) -> bool:
    if value is None:
        return default

    if isinstance(value, bool):
        return value

    text = str(value).strip().lower()

    if text in ("1", "true", "yes", "on", "enable", "enabled"):
        return True

    if text in ("0", "false", "no", "off", "disable", "disabled"):
        return False

    return default


def parse_wind_event_row(row: dict[str, Any]) -> dict[str, Any]:
    def first_float(names: tuple[str, ...], default: float | None = None) -> float:
        for name in names:
            if name in row and row[name] not in ("", None):
                return float(row[name])

        if default is not None:
            return default

        raise KeyError(names[0])

    enabled = bool_from_value(row.get("enabled", row.get("enable_wind", True)), True)

    return {
        "time_s": first_float(("time_s", "time", "t")),
        "x": first_float(("x", "wind_x", "vx")),
        "y": first_float(("y", "wind_y", "vy")),
        "z": first_float(("z", "wind_z", "vz"), 0.0),
        "enabled": enabled,
    }


def parse_wind_events(spec: str) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []

    for item in spec.replace(";", ",").split(","):
        item = item.strip()

        if not item:
            continue

        parts = [part.strip() for part in item.replace("/", ":").split(":")]

        if len(parts) not in (4, 5):
            raise ValueError(f"Invalid wind event '{item}', expected time:x:y:z[:enabled]")

        events.append({
            "time_s": float(parts[0]),
            "x": float(parts[1]),
            "y": float(parts[2]),
            "z": float(parts[3]),
            "enabled": bool_from_value(parts[4], True) if len(parts) == 5 else True,
        })

    return events


def load_wind_events_file(path: Path) -> list[dict[str, Any]]:
    if path.suffix.lower() == ".json":
        raw = json.loads(path.read_text())

        if not isinstance(raw, list):
            raise ValueError(f"{path} must contain a JSON list of wind event objects")

        return [parse_wind_event_row(dict(row)) for row in raw]

    rows: list[dict[str, Any]] = []

    with path.open(newline="") as file:
        sample = file.read(4096)
        file.seek(0)

        try:
            has_header = csv.Sniffer().has_header(sample) if sample.strip() else True
        except csv.Error:
            has_header = True

        if has_header:
            for row in csv.DictReader(line for line in file if not line.lstrip().startswith("#")):
                rows.append(parse_wind_event_row(row))

        else:
            reader = csv.reader(line for line in file if not line.lstrip().startswith("#"))

            for values in reader:
                if len(values) < 4:
                    continue

                rows.append(parse_wind_event_row({
                    "time_s": values[0],
                    "x": values[1],
                    "y": values[2],
                    "z": values[3],
                    "enabled": values[4] if len(values) > 4 else True,
                }))

    return rows


def wind_sidecar_candidates(log_path: Path) -> list[Path]:
    return [
        log_path.with_suffix(".wind.csv"),
        log_path.with_suffix(".wind.json"),
        log_path.with_name(f"{log_path.stem}_wind.csv"),
        log_path.with_name(f"{log_path.stem}_wind.json"),
        Path("/tmp/iris_ftc_wind_events.csv"),
        Path("wind_events.csv"),
        Path("wind_events.json"),
    ]


def parse_initial_wind_from_sdf(path: Path) -> tuple[float, float, float] | None:
    if not path.exists():
        return None

    try:
        root = ET.parse(path).getroot()
    except ET.ParseError:
        return None

    linear_velocity = root.find(".//wind/linear_velocity")

    if linear_velocity is None or not linear_velocity.text:
        return None

    try:
        values = [float(value) for value in linear_velocity.text.split()]
    except ValueError:
        return None

    if len(values) != 3:
        return None

    return values[0], values[1], values[2]


def load_wind_schedule(args: argparse.Namespace, log_path: Path) -> tuple[list[dict[str, Any]], str]:
    events: list[dict[str, Any]] = []
    source = ""

    if args.wind_events:
        events = parse_wind_events(args.wind_events)
        source = "--wind-events"

    elif args.wind_events_file:
        events = load_wind_events_file(Path(args.wind_events_file).expanduser().resolve())
        source = args.wind_events_file

    else:
        for candidate in wind_sidecar_candidates(log_path):
            candidate = candidate.expanduser()

            if candidate.exists():
                events = load_wind_events_file(candidate.resolve())
                source = str(candidate)
                break

    world_path = Path(args.wind_world).expanduser()
    initial_wind = parse_initial_wind_from_sdf(world_path)

    has_initial = any(abs(float(event["time_s"])) < 1e-6 for event in events)

    if initial_wind is not None and not has_initial:
        events.insert(0, {
            "time_s": 0.0,
            "x": initial_wind[0],
            "y": initial_wind[1],
            "z": initial_wind[2],
            "enabled": True,
        })

        if not source:
            source = f"initial wind from {world_path}"

    events.sort(key=lambda event: float(event["time_s"]))
    return events, source


def trace_from_wind_events(
    events: list[dict[str, Any]],
    component: str,
    label: str,
    start: float | None,
    end: float | None,
    max_points: int,
) -> dict[str, Any] | None:
    if not events:
        return None

    x = np.asarray([float(event["time_s"]) for event in events], dtype=np.float64)

    if component == "speed":
        y = np.asarray([
            math.sqrt(float(event["x"]) ** 2 + float(event["y"]) ** 2 + float(event["z"]) ** 2)
            if bool_from_value(event.get("enabled"), True) else 0.0
            for event in events
        ], dtype=np.float64)

    elif component == "enabled":
        y = np.asarray([1.0 if bool_from_value(event.get("enabled"), True) else 0.0 for event in events], dtype=np.float64)

    else:
        y = np.asarray([
            float(event[component]) if bool_from_value(event.get("enabled"), True) else 0.0
            for event in events
        ], dtype=np.float64)

    if start is not None:
        previous = np.flatnonzero(x <= start)

        if len(previous) > 0 and (len(x) == 0 or x[previous[-1]] < start):
            held_value = y[previous[-1]]
            x = np.insert(x, 0, start)
            y = np.insert(y, 0, held_value)

    if end is not None:
        previous = np.flatnonzero(x <= end)

        if len(previous) > 0 and x[previous[-1]] < end:
            x = np.append(x, end)
            y = np.append(y, y[previous[-1]])

    x, y = clip_window(x, y, start, end)
    xp, yp = downsample(x, y, max_points)

    if not xp:
        return None

    return {"x": xp, "y": yp, "label": label, "shape": "hv"}


def build_wind_panel(
    events: list[dict[str, Any]],
    start: float | None,
    end: float | None,
    max_points: int,
) -> dict[str, Any] | None:
    return panel("GZ Wind Command Schedule", "m/s / enabled", [
        trace_from_wind_events(events, "speed", "wind speed", start, end, max_points),
        trace_from_wind_events(events, "x", "wind x", start, end, max_points),
        trace_from_wind_events(events, "y", "wind y", start, end, max_points),
        trace_from_wind_events(events, "z", "wind z", start, end, max_points),
        trace_from_wind_events(events, "enabled", "wind enabled", start, end, max_points),
    ])


def load_aero_debug_csv(path: Path) -> dict[str, np.ndarray] | None:
    if not path.exists():
        return None

    rows: list[dict[str, str]] = []

    with path.open(newline="") as file:
        for row in csv.DictReader(line for line in file if not line.lstrip().startswith("#")):
            rows.append(row)

    if not rows:
        return None

    data: dict[str, np.ndarray] = {}

    for column in rows[0].keys():
        values: list[float] = []

        for row in rows:
            try:
                values.append(float(row[column]))
            except (TypeError, ValueError):
                values.append(float("nan"))

        data[column] = np.asarray(values, dtype=np.float64)

    return data


def aero_trace(
    aero: dict[str, np.ndarray] | None,
    field_name: str,
    label: str,
    start: float | None,
    end: float | None,
    max_points: int,
) -> dict[str, Any] | None:
    if aero is None or "time_s" not in aero or field_name not in aero:
        return None

    return trace_from_arrays(aero["time_s"], aero[field_name], label, start, end, max_points)


def build_aero_panel(
    aero: dict[str, np.ndarray] | None,
    start: float | None,
    end: float | None,
    max_points: int,
) -> dict[str, Any] | None:
    traces = [
        aero_trace(aero, "airspeed", "V airspeed", start, end, max_points),
        aero_trace(aero, "fx_s", "fx_s = Cx V", start, end, max_points),
        aero_trace(aero, "fy_s", "fy_s = sign(r)(Cy1 V + Cy2 V^2)", start, end, max_points),
        aero_trace(aero, "force_x", "force world x", start, end, max_points),
        aero_trace(aero, "force_y", "force world y", start, end, max_points),
        aero_trace(aero, "force_z", "force world z", start, end, max_points),
        aero_trace(aero, "r_about_spin_axis", "r about spin axis", start, end, max_points),
    ]
    result = panel("GZ Paper Aero Force - Stabilization Frame", "m/s, m/s^2, N", traces)

    if result is not None or aero is None or start is None or end is None:
        return result

    # GZ sidecar CSVs use simulation time, while ULog panels use log-relative
    # time. If the selected FTC window misses the sidecar by a small offset, keep
    # the diagnostic visible instead of silently dropping the whole panel.
    return panel("GZ Paper Aero Force - Stabilization Frame", "m/s, m/s^2, N", [
        aero_trace(aero, "airspeed", "V airspeed", None, None, max_points),
        aero_trace(aero, "fx_s", "fx_s = Cx V", None, None, max_points),
        aero_trace(aero, "fy_s", "fy_s = sign(r)(Cy1 V + Cy2 V^2)", None, None, max_points),
        aero_trace(aero, "force_x", "force world x", None, None, max_points),
        aero_trace(aero, "force_y", "force world y", None, None, max_points),
        aero_trace(aero, "force_z", "force world z", None, None, max_points),
        aero_trace(aero, "r_about_spin_axis", "r about spin axis", None, None, max_points),
    ])


def build_aero_moment_panel(
    aero: dict[str, np.ndarray] | None,
    start: float | None,
    end: float | None,
    max_points: int,
) -> dict[str, Any] | None:
    traces = [
        aero_trace(aero, "psi_s", "heading psi_s", start, end, max_points),
        aero_trace(aero, "moment_b_x", "Ma body x", start, end, max_points),
        aero_trace(aero, "moment_b_y", "Ma body y", start, end, max_points),
        aero_trace(aero, "moment_b_z", "Ma body z", start, end, max_points),
        aero_trace(aero, "moment_w_x", "Ma world x", start, end, max_points),
        aero_trace(aero, "moment_w_y", "Ma world y", start, end, max_points),
        aero_trace(aero, "moment_w_z", "Ma world z", start, end, max_points),
    ]
    result = panel("GZ Aero Moment Surrogate - Disabled Unless Configured", "rad / Nm", traces)

    if result is not None or aero is None or start is None or end is None:
        return result

    return panel("GZ Aero Moment Surrogate - Disabled Unless Configured", "rad / Nm", [
        aero_trace(aero, "psi_s", "heading psi_s", None, None, max_points),
        aero_trace(aero, "moment_b_x", "Ma body x", None, None, max_points),
        aero_trace(aero, "moment_b_y", "Ma body y", None, None, max_points),
        aero_trace(aero, "moment_b_z", "Ma body z", None, None, max_points),
        aero_trace(aero, "moment_w_x", "Ma world x", None, None, max_points),
        aero_trace(aero, "moment_w_y", "Ma world y", None, None, max_points),
        aero_trace(aero, "moment_w_z", "Ma world z", None, None, max_points),
    ])


def trace_from_arrays(
    x: np.ndarray,
    y: np.ndarray,
    label: str,
    start: float | None,
    end: float | None,
    max_points: int,
) -> dict[str, Any] | None:
    x, y = clip_window(np.asarray(x, dtype=np.float64), np.asarray(y, dtype=np.float64), start, end)
    xp, yp = downsample(x, y, max_points)

    if not xp:
        return None

    return {"x": xp, "y": yp, "label": label}


def interpolate_field(
    source: dict[str, np.ndarray] | None,
    source_field: str,
    target_t: np.ndarray,
    t0_us: int,
) -> np.ndarray:
    if source is None or source_field not in source or "timestamp" not in source:
        return np.full_like(target_t, np.nan, dtype=np.float64)

    source_t = time_s(source, t0_us)
    source_y = field(source, source_field)

    if source_y is None or len(source_t) < 1:
        return np.full_like(target_t, np.nan, dtype=np.float64)

    if len(source_t) == 1:
        return np.full_like(target_t, float(source_y[0]), dtype=np.float64)

    return np.interp(target_t, source_t, source_y)


def param_timeseries(
    name: str,
    target_t: np.ndarray,
    params: dict[str, Any],
    changed_params: list[tuple[int, str, Any]],
    t0_us: int,
    default: float,
) -> np.ndarray:
    initial = params.get(name, default)

    try:
        current = float(initial)
    except (TypeError, ValueError):
        current = float(default)

    values = np.full_like(target_t, current, dtype=np.float64)
    changes = sorted((timestamp, value) for timestamp, param_name, value in changed_params if param_name == name)

    for timestamp, value in changes:
        try:
            current = float(value)
        except (TypeError, ValueError):
            continue

        change_t = (float(timestamp) - float(t0_us)) * 1e-6
        values[target_t >= change_t] = current

    return values


def hold_fields(
    source: dict[str, np.ndarray] | None,
    field_names: list[str],
    target_t: np.ndarray,
    t0_us: int,
) -> np.ndarray | None:
    if source is None or "timestamp" not in source or not all(name in source for name in field_names):
        return None

    source_t = time_s(source, t0_us)

    if len(source_t) < 1:
        return None

    indices = np.searchsorted(source_t, target_t, side="right") - 1
    valid = indices >= 0
    indices = np.clip(indices, 0, len(source_t) - 1)
    held = np.vstack([field(source, name)[indices] for name in field_names]).T
    held[~valid, :] = np.nan
    return held


def normalize_rows(values: np.ndarray) -> np.ndarray:
    result = np.array(values, dtype=np.float64, copy=True)
    norms = np.linalg.norm(result, axis=1)
    valid = np.isfinite(norms) & (norms > 1e-12)
    result[valid] = result[valid] / norms[valid, None]
    result[~valid] = np.nan
    return result


def quat_to_dcm_array(q: np.ndarray) -> np.ndarray:
    q = normalize_rows(q)
    w = q[:, 0]
    x = q[:, 1]
    y = q[:, 2]
    z = q[:, 3]
    dcm = np.empty((len(q), 3, 3), dtype=np.float64)
    dcm[:, 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    dcm[:, 0, 1] = 2.0 * (x * y - z * w)
    dcm[:, 0, 2] = 2.0 * (x * z + y * w)
    dcm[:, 1, 0] = 2.0 * (x * y + z * w)
    dcm[:, 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    dcm[:, 1, 2] = 2.0 * (y * z - x * w)
    dcm[:, 2, 0] = 2.0 * (x * z - y * w)
    dcm[:, 2, 1] = 2.0 * (y * z + x * w)
    dcm[:, 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return dcm


def compute_pa_ndi_analysis(
    data: dict[str, dict[str, np.ndarray]],
    params: dict[str, Any],
    changed_params: list[tuple[int, str, Any]],
    t0_us: int,
) -> dict[str, np.ndarray]:
    attitude = data.get("vehicle_attitude")
    attitude_sp = data.get("vehicle_attitude_setpoint")
    rates = data.get("vehicle_angular_velocity")
    rates_sp = data.get("vehicle_rates_setpoint")
    debug = data.get("control_allocator_ftc_debug")

    if attitude is None or attitude_sp is None or rates is None:
        return {}

    q = np.vstack([field(attitude, f"q[{i}]") for i in range(4)]).T
    qd_hold = hold_fields(attitude_sp, [f"q_d[{i}]" for i in range(4)], time_s(attitude, t0_us), t0_us)

    if qd_hold is None:
        return {}

    t = time_s(attitude, t0_us)
    R = quat_to_dcm_array(q)
    R_des = quat_to_dcm_array(qd_hold)
    n_des_inertial = -R_des[:, :, 2]
    n_des_inertial = normalize_rows(n_des_inertial)
    h = np.einsum("nij,nj->ni", np.transpose(R, (0, 2, 1)), n_des_inertial)

    # The controller computes n_des_dot only when a new attitude setpoint timestamp arrives.
    # For readability this report holds the latest setpoint-rate derivative across the interval.
    t_sp = time_s(attitude_sp, t0_us)
    qd_sp = np.vstack([field(attitude_sp, f"q_d[{i}]") for i in range(4)]).T
    R_sp = quat_to_dcm_array(qd_sp)
    n_des_sp = normalize_rows(-R_sp[:, :, 2])
    n_des_dot_sp = np.zeros_like(n_des_sp)

    if len(t_sp) > 1:
        dt_sp = np.diff(t_sp)
        dt_sp = np.clip(dt_sp, 0.0002, 0.2)
        n_des_dot_sp[1:] = (n_des_sp[1:] - n_des_sp[:-1]) / dt_sp[:, None]

    indices = np.searchsorted(t_sp, t, side="right") - 1
    valid = indices >= 0
    indices = np.clip(indices, 0, len(t_sp) - 1)
    n_des_dot_inertial = n_des_dot_sp[indices]
    n_des_dot_inertial[~valid] = np.nan
    n_des_dot_body = np.einsum("nij,nj->ni", np.transpose(R, (0, 2, 1)), n_des_dot_inertial)

    rates_t = time_s(rates, t0_us)
    p = interpolate_field(rates, "xyz[0]", t, t0_us)
    q_rate = interpolate_field(rates, "xyz[1]", t, t0_us)
    r = interpolate_field(rates, "xyz[2]", t, t0_us)
    p_sp = interpolate_field(rates_sp, "roll", t, t0_us)
    q_sp = interpolate_field(rates_sp, "pitch", t, t0_us)

    primary_axis = np.vstack([
        param_timeseries("MC_FTC_NX", t, params, changed_params, t0_us, 0.0),
        param_timeseries("MC_FTC_NY", t, params, changed_params, t0_us, 0.0),
        param_timeseries("MC_FTC_NZ", t, params, changed_params, t0_us, -1.0),
    ]).T
    primary_axis = normalize_rows(primary_axis)
    invalid_axis = ~np.all(np.isfinite(primary_axis), axis=1)
    primary_axis[invalid_axis] = np.array([0.0, 0.0, -1.0])
    kx = param_timeseries("MC_FTC_KX", t, params, changed_params, t0_us, 0.0)
    ky = param_timeseries("MC_FTC_KY", t, params, changed_params, t0_us, 0.0)
    h1 = h[:, 0]
    h2 = h[:, 1]
    h3 = h[:, 2]
    h3_limited = np.clip(h3, -1.0, -0.05)
    nu_x = kx * (primary_axis[:, 0] - h1)
    nu_y = ky * (primary_axis[:, 1] - h2)
    yaw_term_p = h1 * r
    yaw_term_q = -h2 * r
    ndot_term_p = -n_des_dot_body[:, 1]
    ndot_term_q = n_des_dot_body[:, 0]
    p_sp_offline = (nu_y + yaw_term_p + ndot_term_p) / h3_limited
    q_sp_offline = -(nu_x + yaw_term_q + ndot_term_q) / h3_limited
    h1_dot_pred = n_des_dot_body[:, 0] - (q_rate * h3 - r * h2)
    h2_dot_pred = n_des_dot_body[:, 1] - (r * h1 - p * h3)
    h1_dot = np.gradient(h1, t, edge_order=1)
    h2_dot = np.gradient(h2, t, edge_order=1)

    active = interpolate_field(debug, "ftc_active", t, t0_us) if debug is not None else np.zeros_like(t)

    return {
        "t": t,
        "active": active,
        "h1": h1,
        "h2": h2,
        "h3": h3,
        "h3_limited": h3_limited,
        "target_nx": primary_axis[:, 0],
        "target_ny": primary_axis[:, 1],
        "target_nz": primary_axis[:, 2],
        "hxy_error": np.hypot(h1 - primary_axis[:, 0], h2 - primary_axis[:, 1]),
        "mc_ftc_kx": kx,
        "mc_ftc_ky": ky,
        "nu_x": nu_x,
        "nu_y": nu_y,
        "yaw_term_p": yaw_term_p,
        "yaw_term_q": yaw_term_q,
        "ndes_dot_body_x": n_des_dot_body[:, 0],
        "ndes_dot_body_y": n_des_dot_body[:, 1],
        "ndot_term_p": ndot_term_p,
        "ndot_term_q": ndot_term_q,
        "pa_p_sp_offline": p_sp_offline,
        "pa_q_sp_offline": q_sp_offline,
        "logged_p_sp": p_sp,
        "logged_q_sp": q_sp,
        "p": p,
        "q": q_rate,
        "r": r,
        "p_tracking_error": p_sp - p,
        "q_tracking_error": q_sp - q_rate,
        "h1_dot": h1_dot,
        "h2_dot": h2_dot,
        "h1_dot_pred": h1_dot_pred,
        "h2_dot_pred": h2_dot_pred,
        "h1_dot_residual": h1_dot - h1_dot_pred,
        "h2_dot_residual": h2_dot - h2_dot_pred,
        "rate_limit": np.full_like(t, math.radians(360.0)),
        "h3_warn": np.full_like(t, -0.5),
        "h3_singular": np.zeros_like(t),
        "rates_sample_interval_ms": np.interp(t, rates_t[1:], np.diff(rates_t) * 1000.0) if len(rates_t) > 1 else np.full_like(t, np.nan),
    }


def interp_trace(
    source: dict[str, np.ndarray] | None,
    source_field: str,
    target: dict[str, np.ndarray] | None,
    target_field: str,
    label: str,
    t0_us: int,
    start: float | None,
    end: float | None,
    max_points: int,
    op: str,
) -> dict[str, Any] | None:
    xs = time_s(source, t0_us) if source is not None and "timestamp" in source else None
    ys = field(source, source_field)
    xt = time_s(target, t0_us) if target is not None and "timestamp" in target else None
    yt = field(target, target_field)

    if xs is None or xt is None or ys is None or yt is None or len(xs) < 2 or len(xt) < 1:
        return None

    target_interp = np.interp(xs, xt, yt)

    if op == "minus":
        y = ys - target_interp
    elif op == "target_minus":
        y = target_interp - ys
    else:
        raise ValueError(op)

    xs, y = clip_window(xs, y, start, end)
    xp, yp = downsample(xs, y, max_points)

    if not xp:
        return None

    return {"x": xp, "y": yp, "label": label}


def dataset_rate(data: dict[str, np.ndarray] | None) -> float | None:
    if data is None or "timestamp" not in data or len(data["timestamp"]) < 2:
        return None
    t = np.asarray(data["timestamp"], dtype=np.float64)
    dt = (t[-1] - t[0]) * 1e-6 / max(len(t) - 1, 1)
    return 1.0 / dt if dt > 0 else None


def active_stats(debug: dict[str, np.ndarray] | None) -> dict[str, Any]:
    if debug is None or "ftc_active" not in debug:
        return {}

    active = np.asarray(debug["ftc_active"], dtype=np.int32) > 0
    stats: dict[str, Any] = {"active_rows": int(np.sum(active))}

    if not np.any(active):
        return stats

    def max_abs(name: str) -> float | None:
        y = field(debug, name)
        if y is None:
            return None
        return float(np.nanmax(np.abs(y[active])))

    stats["success_fraction"] = None

    if "indi_success" in debug:
        success = np.asarray(debug["indi_success"], dtype=np.int32) > 0
        stats["success_fraction"] = float(np.mean(success[active]))

    if "indi_fail_reason" in debug:
        reasons = np.asarray(debug["indi_fail_reason"], dtype=np.int32)[active]
        stats["fail_reasons"] = dict(Counter(int(v) for v in reasons))

    stats["max_abs_nu_in"] = [max_abs(f"indi_nu_in[{i}]") for i in range(3)]
    stats["max_abs_error"] = [max_abs(f"indi_error[{i}]") for i in range(3)]

    for prefix in ("post_ftc_control", "final_control"):
        fractions = []

        for i in range(4):
            y = field(debug, f"{prefix}[{i}]")
            if y is None:
                fractions.append(None)
            else:
                fractions.append(float(np.mean(y[active] >= 0.99)))

        stats[f"{prefix}_sat_fraction"] = fractions

    thrust = field(debug, "thrust_setpoint[2]")
    if thrust is not None:
        stats["min_thrust_sp_z"] = float(np.nanmin(thrust[active]))

    return stats


def pa_active_stats(pa: dict[str, np.ndarray]) -> dict[str, Any]:
    if not pa or "active" not in pa:
        return {}

    active = np.asarray(pa["active"]) > 0.5

    if not np.any(active):
        return {}

    stats: dict[str, Any] = {}

    def first_time(condition: np.ndarray) -> float | None:
        idx = np.flatnonzero(active & condition)
        return None if len(idx) == 0 else float(pa["t"][idx[0]])

    for name in ("h1", "h2", "h3", "hxy_error", "p_tracking_error", "q_tracking_error", "r"):
        values = pa.get(name)

        if values is not None:
            stats[f"pa_{name}_active_min"] = float(np.nanmin(values[active]))
            stats[f"pa_{name}_active_max"] = float(np.nanmax(values[active]))

    stats["pa_first_h3_gt_minus_0p5_s"] = first_time(pa["h3"] > -0.5)
    stats["pa_first_h3_gt_minus_0p3_s"] = first_time(pa["h3"] > -0.3)
    stats["pa_first_h3_gt_0_s"] = first_time(pa["h3"] > 0.0)
    stats["pa_first_hxy_error_gt_0p5_s"] = first_time(pa["hxy_error"] > 0.5)
    stats["pa_first_hxy_error_gt_1_s"] = first_time(pa["hxy_error"] > 1.0)

    logged_p_sp = pa.get("logged_p_sp")
    logged_q_sp = pa.get("logged_q_sp")

    if logged_p_sp is not None:
        stats["pa_p_sp_rate_limit_fraction"] = float(np.mean(np.abs(logged_p_sp[active]) > 6.2))

    if logged_q_sp is not None:
        stats["pa_q_sp_rate_limit_fraction"] = float(np.mean(np.abs(logged_q_sp[active]) > 6.2))

    return stats


def missing_topics(data: dict[str, dict[str, np.ndarray]]) -> list[str]:
    return [name for name in TOPICS if name not in data]


def filtered_param_changes(
    changed_params: list[tuple[int, str, Any]],
    t0_us: int,
) -> list[dict[str, Any]]:
    rows = []

    for timestamp, name, value in changed_params:
        if name not in PARAM_CHANGE_FILTER:
            continue

        rows.append({
            "time_s": (float(timestamp) - float(t0_us)) * 1e-6,
            "name": name,
            "value": value,
        })

    return rows


def pa_trace(pa: dict[str, np.ndarray], name: str, label: str, start: float | None, end: float | None,
             max_points: int) -> dict[str, Any] | None:
    if not pa or "t" not in pa or name not in pa:
        return None
    return trace_from_arrays(pa["t"], pa[name], label, start, end, max_points)


def build_panels(data: dict[str, dict[str, np.ndarray]], params: dict[str, Any], t0_us: int,
                 changed_params: list[tuple[int, str, Any]],
                 start: float | None, end: float | None,
                 max_points: int) -> list[dict[str, Any]]:
    pos = data.get("vehicle_local_position")
    pos_sp = data.get("vehicle_local_position_setpoint")
    thrust = data.get("vehicle_thrust_setpoint")
    torque = data.get("vehicle_torque_setpoint")
    rates_sp = data.get("vehicle_rates_setpoint")
    rates = data.get("vehicle_angular_velocity")
    accel = data.get("vehicle_acceleration")
    motors = data.get("actuator_motors")
    debug = data.get("control_allocator_ftc_debug")
    ftc_physical_sp = data.get("vehicle_ftc_physical_setpoint")
    pa = compute_pa_ndi_analysis(data, params, changed_params, t0_us)

    panels = [
        panel("Position Control - XY Position", "m", [
            make_trace(pos, "x", "x", t0_us, start, end, max_points),
            make_trace(pos_sp, "x", "x_sp", t0_us, start, end, max_points),
            make_trace(pos, "y", "y", t0_us, start, end, max_points),
            make_trace(pos_sp, "y", "y_sp", t0_us, start, end, max_points),
        ]),
        panel("Position Control - Z And Vertical Velocity", "m / m/s", [
            make_trace(pos, "z", "z", t0_us, start, end, max_points),
            make_trace(pos_sp, "z", "z_sp", t0_us, start, end, max_points),
            make_trace(pos, "vz", "vz", t0_us, start, end, max_points),
            make_trace(pos_sp, "vz", "vz_sp", t0_us, start, end, max_points),
        ]),
        panel("Position Control - Accel/Thrust/Fz", "normalized / m/s^2", [
            make_trace(pos_sp, "acceleration[0]", "acc_sp_x", t0_us, start, end, max_points),
            make_trace(pos_sp, "acceleration[1]", "acc_sp_y", t0_us, start, end, max_points),
            make_trace(pos_sp, "acceleration[2]", "acc_sp_z", t0_us, start, end, max_points),
            make_trace(thrust, "xyz[2]", "vehicle_thrust_sp_z", t0_us, start, end, max_points),
            make_trace(ftc_physical_sp, "fz_des_body", "ftc_physical_fz_des_body", t0_us, start, end, max_points),
            make_trace(ftc_physical_sp, "thrust_sp[2]", "ftc_physical_thrust_sp_z", t0_us, start, end, max_points),
            make_trace(debug, "indi_fz_des", "fz_des", t0_us, start, end, max_points),
            make_trace(debug, "indi_y_dot_f[2]", "fz_meas_f", t0_us, start, end, max_points),
            make_trace(debug, "indi_error[2]", "fz_error", t0_us, start, end, max_points),
        ]),
        panel("PA+NDI - Primary Axis Tracking", "axis component", [
            pa_trace(pa, "h1", "h1 = R^T n_des x", start, end, max_points),
            pa_trace(pa, "target_nx", "target nx", start, end, max_points),
            pa_trace(pa, "h2", "h2 = R^T n_des y", start, end, max_points),
            pa_trace(pa, "target_ny", "target ny", start, end, max_points),
            pa_trace(pa, "h3", "h3 = R^T n_des z", start, end, max_points),
            pa_trace(pa, "target_nz", "target nz", start, end, max_points),
            pa_trace(pa, "h3_warn", "h3=-0.5 warning", start, end, max_points),
            pa_trace(pa, "h3_singular", "h3=0 singular", start, end, max_points),
        ]),
        panel("PA+NDI - Primary Axis Error And Validity", "axis / 1", [
            pa_trace(pa, "hxy_error", "sqrt((h1-nx)^2+(h2-ny)^2)", start, end, max_points),
            pa_trace(pa, "h3", "h3 raw", start, end, max_points),
            pa_trace(pa, "h3_limited", "h3 used in code", start, end, max_points),
            pa_trace(pa, "active", "FTC active", start, end, max_points),
        ]),
        panel("PA+NDI - Numerator Terms For p_sp/q_sp", "rad/s numerator", [
            pa_trace(pa, "nu_x", "nu_x = Kx(nx-h1)", start, end, max_points),
            pa_trace(pa, "nu_y", "nu_y = Ky(ny-h2)", start, end, max_points),
            pa_trace(pa, "yaw_term_p", "p numerator yaw h1*r", start, end, max_points),
            pa_trace(pa, "yaw_term_q", "q numerator yaw -h2*r", start, end, max_points),
            pa_trace(pa, "ndot_term_p", "p feedforward -n_dot_body_y", start, end, max_points),
            pa_trace(pa, "ndot_term_q", "q feedforward +n_dot_body_x", start, end, max_points),
        ]),
        panel("PA+NDI - Runtime Parameters Used In Offline Reconstruction", "parameter value", [
            pa_trace(pa, "target_nx", "MC_FTC_NX normalized", start, end, max_points),
            pa_trace(pa, "target_ny", "MC_FTC_NY normalized", start, end, max_points),
            pa_trace(pa, "target_nz", "MC_FTC_NZ normalized", start, end, max_points),
            pa_trace(pa, "mc_ftc_kx", "MC_FTC_KX", start, end, max_points),
            pa_trace(pa, "mc_ftc_ky", "MC_FTC_KY", start, end, max_points),
        ]),
        panel("PA+NDI - Offline Rate Setpoint Reconstruction", "rad/s", [
            pa_trace(pa, "pa_p_sp_offline", "offline p_sp from PA formula", start, end, max_points),
            pa_trace(pa, "logged_p_sp", "logged p_sp", start, end, max_points),
            pa_trace(pa, "p", "actual p", start, end, max_points),
            pa_trace(pa, "pa_q_sp_offline", "offline q_sp from PA formula", start, end, max_points),
            pa_trace(pa, "logged_q_sp", "logged q_sp", start, end, max_points),
            pa_trace(pa, "q", "actual q", start, end, max_points),
            pa_trace(pa, "rate_limit", "360 deg/s limit", start, end, max_points),
        ]),
        panel("PA+NDI - Rate Tracking And Yaw Spin", "rad/s", [
            pa_trace(pa, "p_tracking_error", "p_sp - p", start, end, max_points),
            pa_trace(pa, "q_tracking_error", "q_sp - q", start, end, max_points),
            pa_trace(pa, "r", "yaw rate r", start, end, max_points),
            pa_trace(pa, "logged_p_sp", "p_sp", start, end, max_points),
            pa_trace(pa, "logged_q_sp", "q_sp", start, end, max_points),
        ]),
        panel("PA+NDI - Kinematic Consistency Check", "1/s", [
            pa_trace(pa, "h1_dot", "d(h1)/dt measured", start, end, max_points),
            pa_trace(pa, "h1_dot_pred", "d(h1)/dt predicted", start, end, max_points),
            pa_trace(pa, "h1_dot_residual", "h1 dot residual", start, end, max_points),
            pa_trace(pa, "h2_dot", "d(h2)/dt measured", start, end, max_points),
            pa_trace(pa, "h2_dot_pred", "d(h2)/dt predicted", start, end, max_points),
            pa_trace(pa, "h2_dot_residual", "h2 dot residual", start, end, max_points),
        ]),
        panel("PA+NDI - Desired Axis Feedforward Derivative", "1/s", [
            pa_trace(pa, "ndes_dot_body_x", "n_des_dot_body_x", start, end, max_points),
            pa_trace(pa, "ndes_dot_body_y", "n_des_dot_body_y", start, end, max_points),
            pa_trace(pa, "rates_sample_interval_ms", "angular velocity log dt [ms]", start, end, max_points),
        ]),
        panel("Unused Legacy Allocator Debug - Not Single-Rotor PA+NDI", "legacy fields", [
            make_trace(debug, "y[0]", "legacy_y0", t0_us, start, end, max_points),
            make_trace(debug, "y[1]", "legacy_y1", t0_us, start, end, max_points),
            make_trace(debug, "nu[0]", "legacy_nu0", t0_us, start, end, max_points),
            make_trace(debug, "nu[1]", "legacy_nu1", t0_us, start, end, max_points),
            make_trace(debug, "chi", "legacy_chi", t0_us, start, end, max_points),
        ]),
        panel("PA+NDI - Body Rates", "rad/s", [
            make_trace(rates, "xyz[0]", "p", t0_us, start, end, max_points),
            make_trace(rates_sp, "roll", "p_sp", t0_us, start, end, max_points),
            make_trace(rates, "xyz[1]", "q", t0_us, start, end, max_points),
            make_trace(rates_sp, "pitch", "q_sp", t0_us, start, end, max_points),
            make_trace(rates, "xyz[2]", "r", t0_us, start, end, max_points),
            make_trace(rates_sp, "yaw", "r_sp", t0_us, start, end, max_points),
        ]),
        panel("INDI Virtual Input - Roll/Pitch Physical Domain", "rad/s^2", [
            make_trace(debug, "indi_nu_in[0]", "nu_in_pdot", t0_us, start, end, max_points),
            make_trace(debug, "indi_y_dot_f[0]", "p_dot_f", t0_us, start, end, max_points),
            make_trace(debug, "indi_error[0]", "p_dot_error", t0_us, start, end, max_points),
            make_trace(debug, "indi_nu_in[1]", "nu_in_qdot", t0_us, start, end, max_points),
            make_trace(debug, "indi_y_dot_f[1]", "q_dot_f", t0_us, start, end, max_points),
            make_trace(debug, "indi_error[1]", "q_dot_error", t0_us, start, end, max_points),
        ]),
        panel("INDI Virtual Input - Body Z Specific Force", "m/s^2", [
            make_trace(debug, "indi_nu_in[2]", "nu_in_fz", t0_us, start, end, max_points),
            make_trace(debug, "indi_fz_des", "fz_des", t0_us, start, end, max_points),
            make_trace(debug, "indi_y_dot_f[2]", "fz_meas_f", t0_us, start, end, max_points),
            make_trace(debug, "indi_fz_error_int", "fz_error_int", t0_us, start, end, max_points),
        ]),
        panel("INDI Allocation State", "state", [
            make_trace(debug, "ftc_active", "ftc_active", t0_us, start, end, max_points),
            make_trace(debug, "indi_control_active", "indi_active", t0_us, start, end, max_points),
            make_trace(debug, "indi_success", "indi_success", t0_us, start, end, max_points),
            make_trace(debug, "indi_fail_reason", "fail_reason", t0_us, start, end, max_points),
            make_trace(debug, "ftc_mode", "ftc_mode", t0_us, start, end, max_points),
        ]),
        panel("INDI Allocation - Motor Commands", "command 0..1", [
            make_trace(debug, "pre_ftc_control[0]", "pre_m0", t0_us, start, end, max_points),
            make_trace(debug, "pre_ftc_control[1]", "pre_m1", t0_us, start, end, max_points),
            make_trace(debug, "pre_ftc_control[2]", "pre_m2", t0_us, start, end, max_points),
            make_trace(debug, "pre_ftc_control[3]", "pre_m3", t0_us, start, end, max_points),
            make_trace(debug, "post_ftc_control[0]", "post_m0", t0_us, start, end, max_points),
            make_trace(debug, "post_ftc_control[1]", "post_m1", t0_us, start, end, max_points),
            make_trace(debug, "post_ftc_control[2]", "post_m2", t0_us, start, end, max_points),
            make_trace(debug, "post_ftc_control[3]", "post_m3", t0_us, start, end, max_points),
            make_trace(motors, "control[0]", "act_m0", t0_us, start, end, max_points),
            make_trace(motors, "control[1]", "act_m1", t0_us, start, end, max_points),
            make_trace(motors, "control[2]", "act_m2", t0_us, start, end, max_points),
            make_trace(motors, "control[3]", "act_m3", t0_us, start, end, max_points),
        ]),
        panel("INDI Allocation - Omega Squared", "rad^2/s^2", [
            make_trace(debug, "indi_omega2_f[0]", "omega2_f_m0", t0_us, start, end, max_points),
            make_trace(debug, "indi_omega2_f[1]", "omega2_f_m1", t0_us, start, end, max_points),
            make_trace(debug, "indi_omega2_f[2]", "omega2_f_m2", t0_us, start, end, max_points),
            make_trace(debug, "indi_omega2_f[3]", "omega2_f_m3", t0_us, start, end, max_points),
            make_trace(debug, "indi_omega2_cmd[0]", "omega2_cmd_m0", t0_us, start, end, max_points),
            make_trace(debug, "indi_omega2_cmd[1]", "omega2_cmd_m1", t0_us, start, end, max_points),
            make_trace(debug, "indi_omega2_cmd[2]", "omega2_cmd_m2", t0_us, start, end, max_points),
            make_trace(debug, "indi_omega2_cmd[3]", "omega2_cmd_m3", t0_us, start, end, max_points),
        ]),
        panel("INDI Allocation - Delta Omega Squared And Matrix", "mixed units", [
            make_trace(debug, "indi_delta_omega2[0]", "delta_omega2_h0", t0_us, start, end, max_points),
            make_trace(debug, "indi_delta_omega2[1]", "delta_omega2_h1", t0_us, start, end, max_points),
            make_trace(debug, "indi_delta_omega2[2]", "delta_omega2_h2", t0_us, start, end, max_points),
            make_trace(debug, "indi_scaled_det", "scaled_det", t0_us, start, end, max_points),
            make_trace(debug, "indi_cond_proxy", "cond_proxy", t0_us, start, end, max_points),
        ]),
        panel("Allocator Inputs - Torque/Thrust Setpoints", "normalized", [
            make_trace(torque, "xyz[0]", "torque_x", t0_us, start, end, max_points),
            make_trace(torque, "xyz[1]", "torque_y", t0_us, start, end, max_points),
            make_trace(torque, "xyz[2]", "torque_z", t0_us, start, end, max_points),
            make_trace(thrust, "xyz[0]", "thrust_x", t0_us, start, end, max_points),
            make_trace(thrust, "xyz[1]", "thrust_y", t0_us, start, end, max_points),
            make_trace(thrust, "xyz[2]", "thrust_z", t0_us, start, end, max_points),
        ]),
        panel("Tracking Errors - Position And Velocity", "m / m/s", [
            interp_trace(pos, "x", pos_sp, "x", "x_error", t0_us, start, end, max_points, "minus"),
            interp_trace(pos, "y", pos_sp, "y", "y_error", t0_us, start, end, max_points, "minus"),
            interp_trace(pos, "z", pos_sp, "z", "z_error", t0_us, start, end, max_points, "minus"),
            interp_trace(pos, "vx", pos_sp, "vx", "vx_error", t0_us, start, end, max_points, "minus"),
            interp_trace(pos, "vy", pos_sp, "vy", "vy_error", t0_us, start, end, max_points, "minus"),
            interp_trace(pos, "vz", pos_sp, "vz", "vz_error", t0_us, start, end, max_points, "minus"),
        ]),
        panel("Sensor Rates Sanity - Accel And Angular Accel", "m/s^2 / rad/s^2", [
            make_trace(accel, "xyz[2]", "vehicle_accel_z_logged", t0_us, start, end, max_points),
            make_trace(rates, "xyz_derivative[0]", "p_dot_logged", t0_us, start, end, max_points),
            make_trace(rates, "xyz_derivative[1]", "q_dot_logged", t0_us, start, end, max_points),
            make_trace(rates, "xyz_derivative[2]", "r_dot_logged", t0_us, start, end, max_points),
        ]),
    ]

    return [p for p in panels if p is not None]


def render_html(
    log_path: Path,
    output_path: Path,
    panels: list[dict[str, Any]],
    params: dict[str, Any],
    param_changes: list[dict[str, Any]],
    rates: dict[str, float | None],
    stats: dict[str, Any],
    event_start: float | None,
    event_end: float | None,
    plot_start: float | None,
    plot_end: float | None,
) -> None:
    payload = {
        "panels": panels,
        "colors": COLORS,
        "eventStart": event_start,
        "eventEnd": event_end,
    }

    param_rows = "\n".join(
        f"<tr><td>{html.escape(k)}</td><td>{html.escape(str(v))}</td></tr>"
        for k, v in params.items()
    )
    rate_rows = "\n".join(
        f"<tr><td>{html.escape(k)}</td><td>{'' if v is None else f'{v:.2f} Hz'}</td></tr>"
        for k, v in rates.items()
    )
    change_rows = "\n".join(
        f"<tr><td>{row['time_s']:.3f} s</td><td>{html.escape(str(row['name']))}</td><td>{html.escape(str(row['value']))}</td></tr>"
        for row in param_changes
    ) or "<tr><td colspan='3'>No relevant runtime parameter changes recorded.</td></tr>"

    stats_rows = "\n".join(
        f"<tr><td>{html.escape(k)}</td><td><code>{html.escape(json.dumps(v, sort_keys=True))}</code></td></tr>"
        for k, v in stats.items()
    )

    payload_json = json.dumps(payload)

    html_text = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>FTC INDI Log Report - {html.escape(log_path.name)}</title>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
<style>
body {{ margin: 0; font-family: Inter, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background: #f3f4f6; color: #111827; }}
header {{ padding: 22px 30px; background: #111827; color: white; }}
h1 {{ margin: 0 0 6px; font-size: 22px; }}
.sub {{ color: #d1d5db; font-size: 13px; }}
.wrap {{ max-width: 1180px; margin: 0 auto; padding: 18px; }}
.cards {{ display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 12px; margin-bottom: 14px; }}
.card {{ background: white; border: 1px solid #d1d5db; border-radius: 8px; padding: 12px; overflow: auto; }}
h2 {{ margin: 0 0 8px; font-size: 15px; }}
table {{ border-collapse: collapse; width: 100%; font-size: 12px; }}
td {{ border-bottom: 1px solid #e5e7eb; padding: 4px 6px; vertical-align: top; }}
td:first-child {{ color: #4b5563; white-space: nowrap; }}
.panel {{ margin: 12px 0; }}
.panel-title {{ margin: 18px 0 8px; font-size: 16px; font-weight: 700; }}
.plot-card {{ background: white; border: 1px solid #d1d5db; border-radius: 8px; padding: 8px; }}
.plot {{ width: 100%; height: 420px; }}
code {{ white-space: pre-wrap; font-size: 11px; }}
@media (max-width: 900px) {{ .cards {{ grid-template-columns: 1fr; }} }}
</style>
</head>
<body>
<header>
  <h1>FTC INDI Log Report</h1>
  <div class="sub">{html.escape(str(log_path))}</div>
  <div class="sub">event: {event_start if event_start is not None else 'none'} s to {event_end if event_end is not None else 'none'} s, plotted window: {plot_start} to {plot_end}</div>
</header>
<main class="wrap">
  <div class="cards">
    <div class="card"><h2>Parameters</h2><table>{param_rows}</table></div>
    <div class="card"><h2>Topic Rates In Log</h2><table>{rate_rows}</table></div>
    <div class="card"><h2>Active Stats</h2><table>{stats_rows}</table></div>
    <div class="card"><h2>Runtime Parameter Changes</h2><table>{change_rows}</table></div>
  </div>
  <div id="plots"></div>
</main>
<script>
const payload = {payload_json};
const colors = payload.colors;
const plots = document.getElementById("plots");

function eventShapes() {{
  const shapes = [];

  if (payload.eventStart !== null) {{
    shapes.push({{
      type: "line",
      xref: "x",
      yref: "paper",
      x0: payload.eventStart,
      x1: payload.eventStart,
      y0: 0,
      y1: 1,
      line: {{ color: "#ef4444", width: 1.5, dash: "dash" }},
    }});
  }}

  if (payload.eventEnd !== null) {{
    shapes.push({{
      type: "line",
      xref: "x",
      yref: "paper",
      x0: payload.eventEnd,
      x1: payload.eventEnd,
      y0: 0,
      y1: 1,
      line: {{ color: "#f97316", width: 1.5, dash: "dash" }},
    }});
  }}

  return shapes;
}}

function eventAnnotations() {{
  if (payload.eventStart === null) return [];

  return [{{
    x: payload.eventStart,
    y: 1,
    xref: "x",
    yref: "paper",
    text: "FTC on",
    showarrow: false,
    xanchor: "left",
    yanchor: "bottom",
    font: {{ color: "#ef4444", size: 11 }},
  }}];
}}

payload.panels.forEach((panel, panelIndex) => {{
  const wrapper = document.createElement("section");
  wrapper.className = "panel";

  const title = document.createElement("div");
  title.className = "panel-title";
  title.textContent = panel.title;
  wrapper.appendChild(title);

  const card = document.createElement("div");
  card.className = "plot-card";

  const plot = document.createElement("div");
  plot.className = "plot";
  plot.id = `plot-${{panelIndex}}`;
  card.appendChild(plot);
  wrapper.appendChild(card);
  plots.appendChild(wrapper);

  const traces = panel.traces.map((trace, traceIndex) => ({{
    type: "scatter",
    mode: "lines",
    x: trace.x,
    y: trace.y,
    name: trace.label,
    line: {{ color: colors[traceIndex % colors.length], width: 1.7, shape: trace.shape || "linear" }},
    hovertemplate: "%{{x:.4f}} s<br>%{{y:.6g}}<extra>%{{fullData.name}}</extra>",
  }}));

  Plotly.newPlot(plot.id, traces, {{
    margin: {{ l: 72, r: 42, t: 8, b: 48 }},
    paper_bgcolor: "#fff",
    plot_bgcolor: "#fafafa",
    dragmode: "pan",
    legend: {{
      orientation: "h",
      y: 1.12,
      x: 0,
      bgcolor: "rgba(255,255,255,0.82)",
      font: {{ size: 11 }},
    }},
    xaxis: {{
      title: "time since log start [s]",
      gridcolor: "#e5e7eb",
      zeroline: false,
      rangeslider: {{ visible: false }},
    }},
    yaxis: {{
      title: panel.ylabel,
      gridcolor: "#e5e7eb",
      zeroline: true,
      zerolinecolor: "#94a3b8",
    }},
    shapes: eventShapes(),
    annotations: eventAnnotations(),
    hovermode: "x unified",
  }}, {{
    responsive: true,
    displaylogo: false,
    scrollZoom: true,
    modeBarButtonsToRemove: ["lasso2d", "select2d"],
  }});
}});
</script>
</body>
</html>
"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(html_text)


def main() -> None:
    args = parse_args()
    log_path = Path(args.ulog).expanduser().resolve() if args.ulog else newest_ulog().resolve()
    output_path = Path(args.output).expanduser().resolve()

    print(f"Using ULog: {log_path}")

    ulog = ULog(str(log_path), list(TOPICS))
    data = dataset_map(ulog)
    t0_us = min(int(d["timestamp"][0]) for d in data.values() if "timestamp" in d and len(d["timestamp"]) > 0)
    params = {name: ulog.initial_parameters.get(name, "") for name in PARAMS}
    changed_params = list(getattr(ulog, "changed_parameters", []))
    param_changes = filtered_param_changes(changed_params, t0_us)

    event_start, event_end = find_event(data.get("control_allocator_ftc_debug"), t0_us)

    if args.full or event_start is None or event_end is None:
        plot_start = None
        plot_end = None
    else:
        plot_start = max(0.0, event_start - args.event_padding)
        plot_end = event_end + args.event_padding

    wind_events, wind_source = load_wind_schedule(args, log_path)
    wind_panel = build_wind_panel(wind_events, plot_start, plot_end, args.max_points)
    aero_debug_path = Path(args.aero_debug_file).expanduser().resolve()
    aero_debug = load_aero_debug_csv(aero_debug_path)
    aero_panel = build_aero_panel(aero_debug, plot_start, plot_end, args.max_points)
    aero_moment_panel = build_aero_moment_panel(aero_debug, plot_start, plot_end, args.max_points)
    panels = build_panels(data, params, t0_us, changed_params, plot_start, plot_end, args.max_points)

    if aero_panel is not None:
        panels.insert(0, aero_panel)

    if aero_moment_panel is not None:
        panels.insert(0, aero_moment_panel)

    if wind_panel is not None:
        panels.insert(0, wind_panel)

    rates = {name: dataset_rate(data.get(name)) for name in TOPICS if name in data}
    stats = active_stats(data.get("control_allocator_ftc_debug"))
    stats.update(pa_active_stats(compute_pa_ndi_analysis(data, params, changed_params, t0_us)))

    if wind_events:
        stats["wind_schedule_source"] = wind_source or "manual/unknown"
        stats["wind_schedule_events"] = len(wind_events)
        stats["wind_schedule_max_speed_m_s"] = max(
            math.sqrt(float(event["x"]) ** 2 + float(event["y"]) ** 2 + float(event["z"]) ** 2)
            if bool_from_value(event.get("enabled"), True) else 0.0
            for event in wind_events
        )

    if aero_debug is not None:
        stats["aero_debug_source"] = str(aero_debug_path)
        stats["aero_debug_rows"] = int(len(aero_debug["time_s"]))
        stats["aero_debug_time_range_s"] = [
            float(np.nanmin(aero_debug["time_s"])),
            float(np.nanmax(aero_debug["time_s"])),
        ]
        stats["aero_debug_max_airspeed_m_s"] = float(np.nanmax(aero_debug["airspeed"]))
        stats["aero_debug_max_abs_fx_s_m_s2"] = float(np.nanmax(np.abs(aero_debug["fx_s"])))
        stats["aero_debug_max_abs_fy_s_m_s2"] = float(np.nanmax(np.abs(aero_debug["fy_s"])))

        for name in ("moment_b_x", "moment_b_y", "moment_b_z"):
            if name in aero_debug:
                stats[f"aero_debug_max_abs_{name}_nm"] = float(np.nanmax(np.abs(aero_debug[name])))

    absent_topics = missing_topics(data)

    if absent_topics:
        print("Missing topics:", ", ".join(absent_topics))

    render_html(log_path, output_path, panels, params, param_changes, rates, stats, event_start, event_end, plot_start,
                plot_end)

    print(f"Wrote {output_path}")
    print(f"Panels: {len(panels)}")
    if wind_events:
        print(f"Wind schedule: {wind_source or 'manual/unknown'} ({len(wind_events)} events)")
        if not args.wind_events and not args.wind_events_file and wind_source.startswith("initial wind"):
            print("  Runtime gz wind_cmd is not recorded in ULog; pass --wind-events or a sidecar wind CSV/JSON to plot changes.")
    if aero_debug is not None:
        print(f"Aero debug: {aero_debug_path} ({len(aero_debug['time_s'])} rows)")
    if param_changes:
        print("Runtime parameter changes:")
        for row in param_changes:
            print(f"  {row['time_s']:.3f}s {row['name']} = {row['value']}")
    if event_start is not None:
        print(f"FTC active window: {event_start:.3f}s .. {event_end:.3f}s")
    if stats:
        print(json.dumps(stats, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
