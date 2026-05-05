from __future__ import annotations

import os
import time
import math
import logging
import uuid
from typing import Any

from flask import Flask, jsonify, render_template, request

from osm_obstacles import FeaturePolygons, empty_features, fetch_feature_polygons
from pathfinding import build_bbox, build_geo_grid, run_astar


DEFAULT_MAP_CENTER = {"lat": 36.8065, "lng": 10.1815}  # Tunis
DEFAULT_GOOGLE_MAPS_KEY = "AIzaSyCjWrG1fFiRO9FbC337e4PqeUyvD8ieZ8o"

app = Flask(__name__)
MAX_OBSTACLE_EXTRACTION_DISTANCE_M = 30_000.0
LONG_RANGE_MAX_GRID_DIM = 80

logging.basicConfig(
    level=getattr(logging, os.getenv("DRONE_LOG_LEVEL", "INFO").upper(), logging.INFO),
    format="%(asctime)s %(levelname)s %(message)s",
)
logger = logging.getLogger("drone_pathfinder")


def _parse_point(payload: dict[str, Any], key: str) -> dict[str, float]:
    point = payload.get(key)
    if not isinstance(point, dict):
        raise ValueError(f"Missing '{key}' point.")

    try:
        lat = float(point["lat"])
        lng = float(point["lng"])
    except Exception as exc:  # noqa: BLE001
        raise ValueError(f"Invalid '{key}' coordinates.") from exc

    if not (-90 <= lat <= 90 and -180 <= lng <= 180):
        raise ValueError(f"'{key}' coordinates out of range.")

    return {"lat": lat, "lng": lng}


@app.get("/")
def index() -> str:
    maps_key = os.getenv("GOOGLE_MAPS_API_KEY", DEFAULT_GOOGLE_MAPS_KEY)
    return render_template("index.html", google_maps_api_key=maps_key)


@app.get("/api/health")
def health() -> Any:
    return jsonify({"ok": True})


@app.post("/api/pathfind")
def pathfind() -> Any:
    request_id = uuid.uuid4().hex[:8]
    started_at = time.perf_counter()
    payload = request.get_json(silent=True) or {}
    logger.info("[%s] /api/pathfind payload received", request_id)

    try:
        start = _parse_point(payload, "start")
        end = _parse_point(payload, "end")
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400

    try:
        rows = int(payload.get("rows", 100))
        cols = int(payload.get("cols", 100))
    except (TypeError, ValueError):
        return jsonify({"error": "rows and cols must be integers."}), 400

    rows = max(25, min(rows, 180))
    cols = max(25, min(cols, 180))

    try:
        movement = int(payload.get("movement", 8))
    except (TypeError, ValueError):
        return jsonify({"error": "movement must be 4 or 8."}), 400

    if movement not in (4, 8):
        return jsonify({"error": "movement must be 4 or 8."}), 400

    try:
        difficult_cost = float(payload.get("difficult_cost", 3.0))
    except (TypeError, ValueError):
        return jsonify({"error": "difficult_cost must be numeric."}), 400

    difficult_cost = max(1.0, min(difficult_cost, 15.0))

    direct_distance_m = _haversine_m(
        start["lat"],
        start["lng"],
        end["lat"],
        end["lng"],
    )

    is_long_range = direct_distance_m > MAX_OBSTACLE_EXTRACTION_DISTANCE_M
    bbox_padding, min_span_degrees = _tuned_bbox_params(direct_distance_m, is_long_range)
    bbox = build_bbox(
        start,
        end,
        padding_ratio=bbox_padding,
        min_span_degrees=min_span_degrees,
    )

    obstacle_warning: str | None = None
    obstacle_source = "overpass"
    features: FeaturePolygons = empty_features()

    if is_long_range:
        rows = min(rows, LONG_RANGE_MAX_GRID_DIM)
        cols = min(cols, LONG_RANGE_MAX_GRID_DIM)
        obstacle_source = "long_range_simplified"
        obstacle_warning = (
            f"Long range mode ({int(direct_distance_m)} m): obstacle extraction disabled "
            "for performance. Use intermediate waypoints for high accuracy."
        )
    else:
        obstacle_fetch_t0 = time.perf_counter()
        try:
            features = fetch_feature_polygons(bbox)
            logger.info(
                "[%s] obstacle fetch ok in %.2f ms (buildings=%d water=%d difficult=%d)",
                request_id,
                (time.perf_counter() - obstacle_fetch_t0) * 1000.0,
                len(features.buildings),
                len(features.water),
                len(features.difficult),
            )
        except RuntimeError as exc:
            features = empty_features()
            obstacle_warning = str(exc)
            obstacle_source = "fallback_empty"
            logger.warning(
                "[%s] obstacle fetch failed in %.2f ms: %s",
                request_id,
                (time.perf_counter() - obstacle_fetch_t0) * 1000.0,
                obstacle_warning,
            )

    # Keep default conservative and avoid over-blocking dense city routes.
    building_margin_cells = 0

    grid_t0 = time.perf_counter()
    grid = build_geo_grid(
        bbox=bbox,
        rows=rows,
        cols=cols,
        start=start,
        end=end,
        features=features,
        difficult_cost=difficult_cost,
        building_margin_cells=building_margin_cells,
    )
    blocked_ratio = grid.blocked_cells / max(1, rows * cols)

    if (
        obstacle_source == "overpass"
        and len(features.water) > 0
        and blocked_ratio > 0.88
    ):
        logger.warning(
            "[%s] suspicious blocked ratio %.3f with water=%d; retrying without water polygons",
            request_id,
            blocked_ratio,
            len(features.water),
        )
        retry_features = FeaturePolygons(
            buildings=features.buildings,
            water=[],
            difficult=features.difficult,
        )
        retry_grid = build_geo_grid(
            bbox=bbox,
            rows=rows,
            cols=cols,
            start=start,
            end=end,
            features=retry_features,
            difficult_cost=difficult_cost,
            building_margin_cells=building_margin_cells,
        )
        retry_ratio = retry_grid.blocked_cells / max(1, rows * cols)
        if retry_ratio + 0.05 < blocked_ratio:
            grid = retry_grid
            features = retry_features
            blocked_ratio = retry_ratio
            obstacle_source = "overpass_water_suppressed"
            msg = (
                "Water polygons suppressed due to suspicious over-blocking from map geometry."
            )
            obstacle_warning = f"{obstacle_warning} {msg}".strip() if obstacle_warning else msg

    # Retry strategy for short/medium routes that appear over-blocked or trapped.
    # Typical symptom: found=False with tiny exploration or unusually high blocked ratio.
    retry_attempted = False
    retry_used = False
    retry_reason = ""

    astar_t0 = time.perf_counter()
    result = run_astar(grid=grid, movement=movement)
    astar_ms = (time.perf_counter() - astar_t0) * 1000.0

    first_pass_explored = result.explored_count
    should_retry = (
        not is_long_range
        and not result.path_found
        and (blocked_ratio > 0.45 or result.explored_count <= 15)
    )
    if should_retry:
        retry_attempted = True
        retry_reason = (
            f"first pass blocked_ratio={blocked_ratio:.3f}, explored={first_pass_explored}"
        )
        retry_rows = min(180, max(rows, 140 if direct_distance_m < 1200 else 120))
        retry_cols = min(180, max(cols, 140 if direct_distance_m < 1200 else 120))
        retry_padding = max(0.07, bbox_padding * 0.65)
        retry_min_span = max(0.0018, min_span_degrees * 0.65)
        retry_bbox = build_bbox(
            start,
            end,
            padding_ratio=retry_padding,
            min_span_degrees=retry_min_span,
        )

        retry_grid = build_geo_grid(
            bbox=retry_bbox,
            rows=retry_rows,
            cols=retry_cols,
            start=start,
            end=end,
            features=features,
            difficult_cost=difficult_cost,
            building_margin_cells=0,
        )
        retry_ratio = retry_grid.blocked_cells / max(1, retry_rows * retry_cols)
        retry_astar_t0 = time.perf_counter()
        retry_result = run_astar(grid=retry_grid, movement=movement)
        retry_astar_ms = (time.perf_counter() - retry_astar_t0) * 1000.0

        if (
            retry_result.path_found
            or (not result.path_found and retry_result.explored_count > result.explored_count * 1.8)
            or retry_ratio + 0.1 < blocked_ratio
        ):
            grid = retry_grid
            result = retry_result
            blocked_ratio = retry_ratio
            rows = retry_rows
            cols = retry_cols
            bbox = retry_bbox
            bbox_padding = retry_padding
            min_span_degrees = retry_min_span
            astar_ms = retry_astar_ms
            retry_used = True
            if obstacle_source == "overpass":
                obstacle_source = "overpass_retry_tuned"
            msg = (
                "Retry mode used for dense urban area (tighter bbox + finer grid)."
            )
            obstacle_warning = f"{obstacle_warning} {msg}".strip() if obstacle_warning else msg

    grid_ms = (time.perf_counter() - grid_t0) * 1000.0

    start_used = grid.centers[grid.start[0]][grid.start[1]]
    end_used = grid.centers[grid.end[0]][grid.end[1]]
    start_shift_m = _haversine_m(start["lat"], start["lng"], start_used[0], start_used[1])
    end_shift_m = _haversine_m(end["lat"], end["lng"], end_used[0], end_used[1])
    if start_shift_m > 5.0 or end_shift_m > 5.0:
        snap_msg = (
            f"Start/end snapped to nearest walkable cells "
            f"(start shift: {int(start_shift_m)} m, end shift: {int(end_shift_m)} m)."
        )
        obstacle_warning = f"{obstacle_warning} {snap_msg}".strip() if obstacle_warning else snap_msg

    duration_ms = round((time.perf_counter() - started_at) * 1000, 2)
    logger.info(
        "[%s] done source=%s dist=%.0fm rows=%d cols=%d movement=%d "
        "grid_ms=%.2f astar_ms=%.2f total_ms=%.2f blocked=%d blocked_ratio=%.3f difficult=%d explored=%d found=%s retry_attempted=%s retry_used=%s",
        request_id,
        obstacle_source,
        direct_distance_m,
        rows,
        cols,
        movement,
        grid_ms,
        astar_ms,
        duration_ms,
        grid.blocked_cells,
        blocked_ratio,
        grid.difficult_cells,
        result.explored_count,
        result.path_found,
        retry_attempted,
        retry_used,
    )

    response = {
        "path": [{"lat": lat, "lng": lng} for lat, lng in result.path_latlng],
        "path_found": result.path_found,
        "cost": result.total_cost,
        "distance_m": round(result.distance_m, 2),
        "explored_count": result.explored_count,
        "path_steps": result.path_steps,
        "compute_time_ms": duration_ms,
        "bbox": bbox,
        "obstacles": {
            "buildings": features.buildings,
            "water": features.water,
            "difficult": features.difficult,
        },
        "obstacle_source": obstacle_source,
        "obstacle_warning": obstacle_warning,
        "grid": {
            "rows": rows,
            "cols": cols,
            "blocked_cells": grid.blocked_cells,
            "difficult_cells": grid.difficult_cells,
            "start_cell": {"row": grid.start[0], "col": grid.start[1]},
            "end_cell": {"row": grid.end[0], "col": grid.end[1]},
            "start_used": {"lat": start_used[0], "lng": start_used[1]},
            "end_used": {"lat": end_used[0], "lng": end_used[1]},
        },
        "diagnostics": {
            "request_id": request_id,
            "direct_distance_m": round(direct_distance_m, 2),
            "obstacle_source": obstacle_source,
            "obstacle_warning": obstacle_warning,
            "grid_build_ms": round(grid_ms, 2),
            "astar_ms": round(astar_ms, 2),
            "total_ms": duration_ms,
            "blocked_ratio": round(blocked_ratio, 4),
            "building_margin_cells": building_margin_cells,
            "bbox_padding": round(bbox_padding, 4),
            "min_span_degrees": round(min_span_degrees, 6),
            "retry_attempted": retry_attempted,
            "retry_used": retry_used,
            "retry_reason": retry_reason,
            "first_pass_explored": first_pass_explored,
            "grid": grid.diagnostics,
        },
    }

    return jsonify(response)


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    h = (
        math.sin(dphi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    )
    return 2.0 * r * math.asin(math.sqrt(h))


def _tuned_bbox_params(direct_distance_m: float, is_long_range: bool) -> tuple[float, float]:
    if is_long_range:
        return 0.06, 0.01

    # For short routes, keep bbox tight so streets are represented with better resolution.
    if direct_distance_m <= 250:
        return 0.12, 0.0018
    if direct_distance_m <= 800:
        return 0.16, 0.0024
    if direct_distance_m <= 2000:
        return 0.22, 0.0035
    if direct_distance_m <= 6000:
        return 0.28, 0.0060
    return 0.35, 0.01


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
