#!/usr/bin/env python3
"""Streamlit HDF5 explorer for RoCo Challenge recordings.

Run with:
    streamlit run scripts/roco_hdf5_explorer.py -- --file gearbox_assembly_demos/data_20251127_212217.hdf5

If the file is not found locally, it will be pulled from the Hugging Face dataset
repo. Layout is wide, with a synced timeline slider and stacked video wall so
you can scroll through all camera feeds together.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence

import altair as alt
import numpy as np
import pandas as pd
import streamlit as st
from streamlit.delta_generator import DeltaGenerator

try:
    import h5py  # type: ignore
except Exception as exc:  # pragma: no cover
    raise SystemExit("h5py is required. pip install h5py") from exc

try:
    from huggingface_hub import hf_hub_download
except Exception as exc:  # pragma: no cover
    raise SystemExit(
        "huggingface_hub is required. pip install huggingface_hub"
    ) from exc


@dataclass(frozen=True)
class DatasetRow:
    name: str
    shape: Sequence[int]
    dtype: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RoCo HDF5 web explorer")
    parser.add_argument(
        "--file",
        required=True,
        help="HDF5 file path or HF repo filename (e.g., gearbox_assembly_demos/data_20251127_212217.hdf5)",
    )
    parser.add_argument(
        "--repo",
        default="rocochallenge2025/rocochallenge2025",
        help="Hugging Face dataset repo id.",
    )
    parser.add_argument(
        "--local-dir",
        default="data/cache",
        help="Local cache directory for downloads.",
    )
    return parser.parse_args()


def download_if_needed(repo: str, fname: str, local_dir: str) -> Path:
    candidate = Path(fname)
    if candidate.exists():
        return candidate
    path = hf_hub_download(
        repo_id=repo,
        filename=fname,
        local_dir=local_dir,
        repo_type="dataset",
    )
    return Path(path)


def list_datasets(handle: h5py.File) -> List[DatasetRow]:
    rows: List[DatasetRow] = []

    def _visitor(name: str, obj: h5py.Dataset) -> None:
        if isinstance(obj, h5py.Dataset):
            rows.append(DatasetRow(name=name, shape=obj.shape, dtype=str(obj.dtype)))

    handle.visititems(_visitor)
    return sorted(rows, key=lambda x: x.name)


@st.cache_data(show_spinner=False)
def describe_file(path: str) -> List[DatasetRow]:
    p = Path(path)
    with h5py.File(p, "r") as handle:
        return list_datasets(handle)


@st.cache_data(show_spinner=False)
def load_array(path: str, key: str) -> np.ndarray:
    """Load a full dataset into memory (cached)."""
    p = Path(path)
    with h5py.File(p, "r") as handle:
        data = np.array(handle[key])
    return data


def normalize_to_uint8(frame: np.ndarray) -> np.ndarray:
    arr = np.asarray(frame)
    arr = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
    if arr.ndim == 2:  # depth without channel
        arr = arr[..., None]
    if arr.dtype == np.uint8:
        out = arr
    else:
        arr = arr.astype("float32")
        mn, mx = float(arr.min()), float(arr.max())
        if mx > mn:
            arr = (arr - mn) / (mx - mn)
        else:
            arr = np.zeros_like(arr)
        out = np.clip(arr * 255.0, 0, 255).astype("uint8")

    if out.shape[-1] == 4:
        out = out[..., :3]
    if out.shape[-1] == 1:
        out = np.repeat(out, 3, axis=-1)
    return out


def pick_image_keys(rows: Iterable[DatasetRow]) -> List[str]:
    keys: List[str] = []
    for r in rows:
        if len(r.shape) == 4 and r.shape[-1] in (1, 3):
            keys.append(r.name)
        elif len(r.shape) == 3 and r.shape[-1] not in (1, 3):
            # depth (T, H, W)
            keys.append(r.name)
    return keys


def pick_timeseries_keys(rows: Iterable[DatasetRow]) -> List[str]:
    keys: List[str] = []
    for r in rows:
        if len(r.shape) == 1:
            keys.append(r.name)
        if len(r.shape) == 2 and r.shape[1] < 64:  # keep charts readable
            keys.append(r.name)
    return keys


def build_ts_chart(series: np.ndarray, idx: int, title: str) -> alt.Chart:
    """(Deprecated) kept for compatibility; unused in overlay version."""
    if series.ndim == 1:
        df = pd.DataFrame({"t": np.arange(series.shape[0]), "value": series})
        rule_df = pd.DataFrame({"t": [idx]})
        line = alt.Chart(df).mark_line().encode(x="t", y="value")
        rule = alt.Chart(rule_df).mark_rule(color="red").encode(x="t")
        return (line + rule).properties(title=title)
    if series.ndim == 2:
        df = (
            pd.DataFrame(series)
            .reset_index()
            .melt(id_vars="index", var_name="channel", value_name="value")
            .rename(columns={"index": "t"})
        )
        rule_df = pd.DataFrame({"t": [idx]})
        line = (
            alt.Chart(df)
            .mark_line()
            .encode(x="t", y="value", color="channel")
        )
        rule = alt.Chart(rule_df).mark_rule(color="red").encode(x="t")
        return (line + rule).properties(title=title)
    return alt.Chart()  # unsupported


def build_overlay_chart(path: str, keys: List[str], frame_idx: int, normalize: bool) -> tuple[alt.Chart, List[tuple[str, str]]]:
    rows: List[dict] = []
    peeks: List[tuple[str, str]] = []

    def _normalize(arr: np.ndarray) -> np.ndarray:
        arr = np.nan_to_num(arr.astype("float32"), nan=0.0, posinf=0.0, neginf=0.0)
        mn, mx = float(arr.min()), float(arr.max())
        if mx > mn:
            return (arr - mn) / (mx - mn)
        return np.zeros_like(arr)

    for key in keys:
        series = load_array(path, key)
        if series.ndim == 1:
            vals = series.astype("float32")
            if normalize:
                vals = _normalize(vals)
            rows.extend([{"t": i, "value": v, "series": key} for i, v in enumerate(vals)])
            if 0 <= frame_idx < vals.shape[0]:
                peeks.append((key, f"{vals[frame_idx]:.4f}"))
        elif series.ndim == 2:
            for ch in range(series.shape[1]):
                label = f"{key}[{ch}]"
                vals = series[:, ch].astype("float32")
                if normalize:
                    vals = _normalize(vals)
                rows.extend([{"t": i, "value": v, "series": label} for i, v in enumerate(vals)])
                if 0 <= frame_idx < vals.shape[0]:
                    peeks.append((label, f"{vals[frame_idx]:.4f}"))
        else:
            continue

    if not rows:
        return alt.Chart(), []

    df = pd.DataFrame(rows)
    rule_df = pd.DataFrame({"t": [frame_idx]})

    line = alt.Chart(df).mark_line().encode(
        x=alt.X("t:Q", title="frame"),
        y=alt.Y("value:Q", title="value (normalized)" if normalize else "value"),
        color="series:N",
    )
    rule = alt.Chart(rule_df).mark_rule(color="red").encode(x="t:Q")

    chart = (line + rule).properties(height=360)
    return chart, peeks


def infer_total_frames(path: str, image_keys: List[str], ts_keys: List[str]) -> int:
    """Use the smallest length across selected series to keep everything in range."""
    lengths: List[int] = []
    for key in image_keys:
        try:
            lengths.append(load_array(path, key).shape[0])
        except Exception:
            continue
    for key in ts_keys:
        try:
            lengths.append(load_array(path, key).shape[0])
        except Exception:
            continue
    return max(1, min(lengths) if lengths else 1)


def current_time_value(path: str, frame_idx: int) -> float | None:
    """If a current_time dataset exists, surface it for the global timeline."""
    try:
        arr = load_array(path, "current_time")
        if 0 <= frame_idx < arr.shape[0]:
            return float(arr[frame_idx])
    except Exception:
        return None
    return None


def advance_frame_if_due(total_frames: int) -> None:
    """Advance frame based on elapsed time; run before widgets are instantiated."""
    if "frame_idx" not in st.session_state:
        st.session_state.frame_idx = 0
    if "play_fps" not in st.session_state:
        st.session_state.play_fps = 10
    if "playing" not in st.session_state:
        st.session_state.playing = False
    if "last_tick" not in st.session_state:
        st.session_state.last_tick = time.time()

    if not st.session_state.playing:
        return

    fps = max(1, int(st.session_state.play_fps))
    now = time.time()
    interval = 1.0 / fps
    if now - st.session_state.last_tick >= interval:
        st.session_state.frame_idx = (int(st.session_state.frame_idx) + 1) % total_frames
        st.session_state.last_tick = now


def sync_frame_slider(total_frames: int) -> int:
    """Shared frame slider + play/stop toggle."""
    st.subheader("Timeline")
    cols = st.columns([3, 1, 1])

    with cols[0]:
        frame = st.slider("Frame", 0, total_frames - 1, st.session_state.frame_idx, key="frame_idx")
    with cols[1]:
        st.number_input("FPS", min_value=1, max_value=60, value=int(st.session_state.play_fps), step=1, key="play_fps")
    with cols[2]:
        if st.button("⏯ Play/Pause"):
            st.session_state.playing = not st.session_state.playing
            st.session_state.last_tick = time.time()
            st.rerun()

    return frame


def schedule_next_tick() -> None:
    """If playing, schedule the next rerun at the desired FPS."""
    if not st.session_state.get("playing", False):
        return
    fps = max(1, int(st.session_state.get("play_fps", 10)))
    time.sleep(1.0 / fps)
    st.rerun()


def render_video_wall(
    container: DeltaGenerator,
    path: str,
    keys: List[str],
    frame_idx: int,
) -> None:
    if not keys:
        container.info("Select at least one image/depth key.")
        return
    with container:
        for key in keys:
            data = load_array(path, key)
            if frame_idx >= data.shape[0]:
                container.warning(f"{key}: frame {frame_idx} out of range.")
                continue
            disp = normalize_to_uint8(data[frame_idx])
            container.image(
                disp,
                caption=f"{key} | frame {frame_idx}/{data.shape[0]-1}",
                width="stretch",
            )


def render_timeseries_panel(
    container: DeltaGenerator,
    path: str,
    keys: List[str],
    frame_idx: int,
) -> None:
    if not keys:
        container.info("Select at least one time-series key.")
        return
    with container:
        normalize = st.checkbox("Normalize (per series)", value=True, key="ts_normalize")
        chart, peeks = build_overlay_chart(path, keys, frame_idx, normalize)
        container.altair_chart(chart, width="stretch")
        if peeks:
            container.dataframe(
                {"series": [k for k, _ in peeks], "value@frame": [v for _, v in peeks]},
                hide_index=True,
                width="stretch",
            )


def app(args: argparse.Namespace) -> None:
    st.set_page_config(page_title="RoCo HDF5 Explorer", layout="wide")
    st.title("RoCo HDF5 Explorer")
    st.caption("Inspect RGB/depth frames and time-series (actions, joints, scores).")

    resolved_path = download_if_needed(args.repo, args.file, args.local_dir)
    rows = describe_file(str(resolved_path))

    st.sidebar.write("**Dataset metadata**")
    st.sidebar.code(str(resolved_path))
    st.sidebar.write(f"{len(rows)} dataset key(s)")

    image_keys = pick_image_keys(rows)
    ts_keys = pick_timeseries_keys(rows)

    if not image_keys and not ts_keys:
        st.error("No compatible datasets found to visualize.")
        st.stop()

    st.sidebar.subheader("Selections")
    selected_images = st.sidebar.multiselect(
        "Image/Depth keys (synced wall)",
        options=image_keys,
        default=image_keys[:3] if image_keys else [],
    )
    selected_ts = st.sidebar.multiselect(
        "Time-series keys (synced charts)",
        options=ts_keys,
        default=ts_keys[:4] if ts_keys else [],
    )

    total_frames = infer_total_frames(str(resolved_path), selected_images, selected_ts)
    advance_frame_if_due(total_frames)
    frame_idx = sync_frame_slider(total_frames)

    # Global timestamp peek if available.
    ts_value = current_time_value(str(resolved_path), frame_idx)
    if ts_value is not None:
        st.caption(f"current_time[{frame_idx}] = {ts_value:.3f}s")

    cols = st.columns([1.1, 1])
    with cols[0]:
        st.subheader("Synced video wall")
        render_video_wall(
            container=st.container(height=720),
            path=str(resolved_path),
            keys=selected_images,
            frame_idx=frame_idx,
        )
    with cols[1]:
        st.subheader("Synced time-series")
        render_timeseries_panel(
            container=st.container(height=720),
            path=str(resolved_path),
            keys=selected_ts,
            frame_idx=frame_idx,
        )

    with st.expander("All dataset keys", expanded=False):
        st.dataframe(
            {
                "name": [r.name for r in rows],
                "shape": [list(r.shape) for r in rows],
                "dtype": [r.dtype for r in rows],
            },
            hide_index=True,
            width="stretch",
        )

    # Advance playback if requested.
    schedule_next_tick()


def main() -> None:
    args = parse_args()
    app(args)


if __name__ == "__main__":
    main()
