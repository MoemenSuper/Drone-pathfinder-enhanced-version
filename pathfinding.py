from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass
from typing import Iterable

from osm_obstacles import FeaturePolygons, point_in_polygon


@dataclass
class GeoGrid:
    bbox: dict[str, float]
    rows: int
    cols: int
    start: tuple[int, int]
    end: tuple[int, int]
    walkable: list[list[bool]]
    costs: list[list[float]]
    centers: list[list[tuple[float, float]]]
    blocked_cells: int
    difficult_cells: int
    diagnostics: dict[str, float | int]


@dataclass
class AStarResult:
    path_found: bool
    path_steps: int
    total_cost: float
    explored_count: int
    distance_m: float
    path_latlng: list[tuple[float, float]]


@dataclass
class PolygonEntry:
    polygon: list[tuple[float, float]]
    bbox: tuple[float, float, float, float]  # lat_min, lat_max, lng_min, lng_max


@dataclass
class SpatialPolygonIndex:
    entries: list[PolygonEntry]
    bins: list[list[list[int]]]
    bin_rows: int
    bin_cols: int
    bbox: dict[str, float]


def build_bbox(
    start: dict[str, float],
    end: dict[str, float],
    padding_ratio: float = 0.3,
    min_span_degrees: float = 0.008,
) -> dict[str, float]:
    lat_min = min(start["lat"], end["lat"])
    lat_max = max(start["lat"], end["lat"])
    lng_min = min(start["lng"], end["lng"])
    lng_max = max(start["lng"], end["lng"])

    lat_span = max(lat_max - lat_min, min_span_degrees)
    lng_span = max(lng_max - lng_min, min_span_degrees)

    lat_pad = lat_span * padding_ratio
    lng_pad = lng_span * padding_ratio

    south = max(-90.0, lat_min - lat_pad)
    north = min(90.0, lat_max + lat_pad)
    west = max(-180.0, lng_min - lng_pad)
    east = min(180.0, lng_max + lng_pad)

    return {
        "south": south,
        "north": north,
        "west": west,
        "east": east,
    }


def latlng_to_cell(lat: float, lng: float, bbox: dict[str, float], rows: int, cols: int) -> tuple[int, int]:
    lat_fraction = (bbox["north"] - lat) / max(1e-12, bbox["north"] - bbox["south"])
    lng_fraction = (lng - bbox["west"]) / max(1e-12, bbox["east"] - bbox["west"])

    row = int(lat_fraction * rows)
    col = int(lng_fraction * cols)

    row = max(0, min(rows - 1, row))
    col = max(0, min(cols - 1, col))
    return row, col


def cell_center(row: int, col: int, bbox: dict[str, float], rows: int, cols: int) -> tuple[float, float]:
    lat_step = (bbox["north"] - bbox["south"]) / rows
    lng_step = (bbox["east"] - bbox["west"]) / cols

    lat = bbox["north"] - (row + 0.5) * lat_step
    lng = bbox["west"] + (col + 0.5) * lng_step
    return lat, lng


def build_geo_grid(
    bbox: dict[str, float],
    rows: int,
    cols: int,
    start: dict[str, float],
    end: dict[str, float],
    features: FeaturePolygons,
    difficult_cost: float,
    building_margin_cells: int = 0,
) -> GeoGrid:
    t0 = time.perf_counter()
    walkable = [[True for _ in range(cols)] for _ in range(rows)]
    costs = [[1.0 for _ in range(cols)] for _ in range(rows)]
    centers = [[(0.0, 0.0) for _ in range(cols)] for _ in range(rows)]
    building_blocked = [[False for _ in range(cols)] for _ in range(rows)]

    blocked_cells = 0
    difficult_cells = 0
    lat_step = (bbox["north"] - bbox["south"]) / rows
    lng_step = (bbox["east"] - bbox["west"]) / cols
    lat_half = lat_step / 2.0
    lng_half = lng_step / 2.0

    building_entries = _build_polygon_entries(features.buildings)
    water_entries = _build_polygon_entries(features.water)
    difficult_entries = _build_polygon_entries(features.difficult)
    building_index = _build_spatial_index(building_entries, bbox)
    water_index = _build_spatial_index(water_entries, bbox)
    difficult_index = _build_spatial_index(difficult_entries, bbox)

    diagnostics: dict[str, float | int] = {
        "cells_total": rows * cols,
        "building_entries": len(building_entries),
        "water_entries": len(water_entries),
        "difficult_entries": len(difficult_entries),
        "building_candidate_checks": 0,
        "water_candidate_checks": 0,
        "difficult_candidate_checks": 0,
    }

    for row in range(rows):
        for col in range(cols):
            lat, lng = cell_center(row, col, bbox, rows, cols)
            centers[row][col] = (lat, lng)
            cell_bounds = (lat - lat_half, lat + lat_half, lng - lng_half, lng + lng_half)
            sample_points = _cell_sample_points(lat, lng, lat_half, lng_half)

            blocked_building, building_checks = _cell_hits_any_polygon(sample_points, cell_bounds, building_index)
            diagnostics["building_candidate_checks"] += building_checks

            blocked_water = False
            water_checks = 0
            if not blocked_building:
                blocked_water, water_checks = _cell_hits_any_polygon(sample_points, cell_bounds, water_index)
            diagnostics["water_candidate_checks"] += water_checks
            blocked = blocked_building or blocked_water
            if blocked:
                walkable[row][col] = False
                costs[row][col] = math.inf
                blocked_cells += 1
                if blocked_building:
                    building_blocked[row][col] = True
                continue

            difficult_hit, difficult_checks = _cell_hits_any_polygon(sample_points, cell_bounds, difficult_index)
            diagnostics["difficult_candidate_checks"] += difficult_checks
            if difficult_hit:
                costs[row][col] = difficult_cost
                difficult_cells += 1

    start_cell = latlng_to_cell(start["lat"], start["lng"], bbox, rows, cols)
    end_cell = latlng_to_cell(end["lat"], end["lng"], bbox, rows, cols)

    if building_margin_cells > 0:
        margin_added = _dilate_buildings(
            walkable=walkable,
            costs=costs,
            building_blocked=building_blocked,
            rows=rows,
            cols=cols,
            radius=building_margin_cells,
            protected_cells={start_cell, end_cell},
        )
        blocked_cells += margin_added
        diagnostics["building_margin_cells"] = building_margin_cells
        diagnostics["building_margin_added_cells"] = margin_added

    # Snap start/end to nearest walkable cell instead of cutting through obstacles.
    snapped_start = _nearest_walkable_cell(walkable, start_cell, max_radius=12)
    snapped_end = _nearest_walkable_cell(walkable, end_cell, max_radius=12)

    if snapped_start is not None:
        start_cell = snapped_start
    else:
        row, col = start_cell
        walkable[row][col] = True
        costs[row][col] = 1.0
        blocked_cells = max(0, blocked_cells - 1)

    if snapped_end is not None:
        end_cell = snapped_end
    else:
        row, col = end_cell
        walkable[row][col] = True
        costs[row][col] = 1.0
        blocked_cells = max(0, blocked_cells - 1)

    return GeoGrid(
        bbox=bbox,
        rows=rows,
        cols=cols,
        start=start_cell,
        end=end_cell,
        walkable=walkable,
        costs=costs,
        centers=centers,
        blocked_cells=blocked_cells,
        difficult_cells=difficult_cells,
        diagnostics={
            **diagnostics,
            "grid_build_ms": round((time.perf_counter() - t0) * 1000.0, 2),
        },
    )


def run_astar(grid: GeoGrid, movement: int = 8) -> AStarResult:
    start = grid.start
    goal = grid.end

    neighbors = _neighbor_offsets(movement)

    open_heap: list[tuple[float, int, tuple[int, int]]] = []
    counter = 0

    g_score: dict[tuple[int, int], float] = {start: 0.0}
    f_start = _heuristic(start, goal, movement)
    heapq.heappush(open_heap, (f_start, counter, start))

    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    closed: set[tuple[int, int]] = set()
    explored_count = 0

    while open_heap:
        _, _, current = heapq.heappop(open_heap)

        if current in closed:
            continue

        closed.add(current)
        explored_count += 1

        if current == goal:
            path_cells = _reconstruct_path(came_from, current)
            path_latlng = [grid.centers[r][c] for r, c in path_cells]
            total_cost = round(g_score[current], 3)
            distance_m = _polyline_distance_m(path_latlng)
            return AStarResult(
                path_found=True,
                path_steps=max(0, len(path_cells) - 1),
                total_cost=total_cost,
                explored_count=explored_count,
                distance_m=distance_m,
                path_latlng=path_latlng,
            )

        current_g = g_score[current]

        for dr, dc, move_weight in neighbors:
            nr = current[0] + dr
            nc = current[1] + dc

            if not (0 <= nr < grid.rows and 0 <= nc < grid.cols):
                continue
            if not grid.walkable[nr][nc]:
                continue
            # Prevent diagonal corner-cutting through blocked cells.
            if dr != 0 and dc != 0:
                if not grid.walkable[current[0] + dr][current[1]]:
                    continue
                if not grid.walkable[current[0]][current[1] + dc]:
                    continue

            neighbor = (nr, nc)
            tentative_g = current_g + (grid.costs[nr][nc] * move_weight)

            if tentative_g < g_score.get(neighbor, math.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                counter += 1
                f = tentative_g + _heuristic(neighbor, goal, movement)
                heapq.heappush(open_heap, (f, counter, neighbor))

    return AStarResult(
        path_found=False,
        path_steps=0,
        total_cost=0.0,
        explored_count=explored_count,
        distance_m=0.0,
        path_latlng=[],
    )


def _build_polygon_entries(polygons: list[list[tuple[float, float]]]) -> list[PolygonEntry]:
    entries: list[PolygonEntry] = []
    for polygon in polygons:
        if len(polygon) < 3:
            continue
        entries.append(PolygonEntry(polygon=polygon, bbox=_polygon_bbox(polygon)))
    return entries


def _polygon_bbox(polygon: list[tuple[float, float]]) -> tuple[float, float, float, float]:
    lat_values = [lat for lat, _ in polygon]
    lng_values = [lng for _, lng in polygon]
    return min(lat_values), max(lat_values), min(lng_values), max(lng_values)


def _cell_sample_points(
    center_lat: float,
    center_lng: float,
    lat_half: float,
    lng_half: float,
) -> list[tuple[float, float]]:
    lat_off = lat_half * 0.45
    lng_off = lng_half * 0.45
    return [
        (center_lat, center_lng),  # center
        (center_lat - lat_off, center_lng - lng_off),  # corners (inset)
        (center_lat - lat_off, center_lng + lng_off),
        (center_lat + lat_off, center_lng - lng_off),
        (center_lat + lat_off, center_lng + lng_off),
        (center_lat - lat_off, center_lng),  # edge midpoints
        (center_lat + lat_off, center_lng),
        (center_lat, center_lng - lng_off),
        (center_lat, center_lng + lng_off),
    ]


def _cell_hits_any_polygon(
    sample_points: list[tuple[float, float]],
    cell_bounds: tuple[float, float, float, float],  # lat_min, lat_max, lng_min, lng_max
    index: SpatialPolygonIndex,
) -> tuple[bool, int]:
    cell_lat_min, cell_lat_max, cell_lng_min, cell_lng_max = cell_bounds
    entries = _index_query(index, cell_bounds)
    checks = 0

    for entry in entries:
        checks += 1
        poly_lat_min, poly_lat_max, poly_lng_min, poly_lng_max = entry.bbox
        if not _bboxes_intersect(
            cell_lat_min,
            cell_lat_max,
            cell_lng_min,
            cell_lng_max,
            poly_lat_min,
            poly_lat_max,
            poly_lng_min,
            poly_lng_max,
        ):
            continue

        # Tiny polygons can be fully inside a cell without covering sample points.
        for v_lat, v_lng in entry.polygon:
            if cell_lat_min <= v_lat <= cell_lat_max and cell_lng_min <= v_lng <= cell_lng_max:
                return True, checks

        for s_lat, s_lng in sample_points:
            if point_in_polygon(s_lat, s_lng, entry.polygon):
                return True, checks

        # Polygon can cross the cell without containing sample points/vertices.
        if _polygon_edges_intersect_cell(entry.polygon, cell_bounds):
            return True, checks

    return False, checks


def _nearest_walkable_cell(
    walkable: list[list[bool]],
    origin: tuple[int, int],
    max_radius: int = 10,
) -> tuple[int, int] | None:
    rows = len(walkable)
    cols = len(walkable[0]) if rows else 0
    o_row, o_col = origin

    if 0 <= o_row < rows and 0 <= o_col < cols and walkable[o_row][o_col]:
        return origin

    best: tuple[int, int] | None = None
    best_dist = math.inf

    for radius in range(1, max_radius + 1):
        row_min = max(0, o_row - radius)
        row_max = min(rows - 1, o_row + radius)
        col_min = max(0, o_col - radius)
        col_max = min(cols - 1, o_col + radius)

        found_this_ring = False
        for r in range(row_min, row_max + 1):
            for c in range(col_min, col_max + 1):
                if not walkable[r][c]:
                    continue
                d = abs(r - o_row) + abs(c - o_col)
                if d < best_dist:
                    best = (r, c)
                    best_dist = d
                    found_this_ring = True
        if found_this_ring:
            return best

    return best


def _dilate_buildings(
    walkable: list[list[bool]],
    costs: list[list[float]],
    building_blocked: list[list[bool]],
    rows: int,
    cols: int,
    radius: int,
    protected_cells: set[tuple[int, int]],
) -> int:
    if radius <= 0:
        return 0

    to_block: set[tuple[int, int]] = set()
    for r in range(rows):
        for c in range(cols):
            if not building_blocked[r][c]:
                continue
            r_min = max(0, r - radius)
            r_max = min(rows - 1, r + radius)
            c_min = max(0, c - radius)
            c_max = min(cols - 1, c + radius)
            for rr in range(r_min, r_max + 1):
                for cc in range(c_min, c_max + 1):
                    if abs(rr - r) + abs(cc - c) > radius:
                        continue
                    if (rr, cc) in protected_cells:
                        continue
                    to_block.add((rr, cc))

    added = 0
    for r, c in to_block:
        if not walkable[r][c]:
            continue
        walkable[r][c] = False
        costs[r][c] = math.inf
        added += 1
    return added


def _build_spatial_index(
    entries: list[PolygonEntry],
    bbox: dict[str, float],
    max_bins: int = 28,
) -> SpatialPolygonIndex:
    if not entries:
        return SpatialPolygonIndex(entries=[], bins=[[[]]], bin_rows=1, bin_cols=1, bbox=bbox)

    # Keep index compact while providing enough pruning.
    bin_rows = max(6, min(max_bins, int(math.sqrt(len(entries) / 6.0)) + 6))
    bin_cols = bin_rows
    bins: list[list[list[int]]] = [[[] for _ in range(bin_cols)] for _ in range(bin_rows)]

    for i, entry in enumerate(entries):
        lat_min, lat_max, lng_min, lng_max = entry.bbox
        r0, c0 = _latlng_to_bin(lat_max, lng_min, bbox, bin_rows, bin_cols)
        r1, c1 = _latlng_to_bin(lat_min, lng_max, bbox, bin_rows, bin_cols)

        row_start = min(r0, r1)
        row_end = max(r0, r1)
        col_start = min(c0, c1)
        col_end = max(c0, c1)

        for r in range(row_start, row_end + 1):
            for c in range(col_start, col_end + 1):
                bins[r][c].append(i)

    return SpatialPolygonIndex(
        entries=entries,
        bins=bins,
        bin_rows=bin_rows,
        bin_cols=bin_cols,
        bbox=bbox,
    )


def _index_query(
    index: SpatialPolygonIndex,
    cell_bounds: tuple[float, float, float, float],
) -> list[PolygonEntry]:
    if not index.entries:
        return []

    lat_min, lat_max, lng_min, lng_max = cell_bounds
    r0, c0 = _latlng_to_bin(lat_max, lng_min, index.bbox, index.bin_rows, index.bin_cols)
    r1, c1 = _latlng_to_bin(lat_min, lng_max, index.bbox, index.bin_rows, index.bin_cols)

    row_start = min(r0, r1)
    row_end = max(r0, r1)
    col_start = min(c0, c1)
    col_end = max(c0, c1)

    seen: set[int] = set()
    candidates: list[PolygonEntry] = []
    for r in range(row_start, row_end + 1):
        for c in range(col_start, col_end + 1):
            for idx in index.bins[r][c]:
                if idx in seen:
                    continue
                seen.add(idx)
                candidates.append(index.entries[idx])
    return candidates


def _latlng_to_bin(
    lat: float,
    lng: float,
    bbox: dict[str, float],
    bin_rows: int,
    bin_cols: int,
) -> tuple[int, int]:
    lat_fraction = (bbox["north"] - lat) / max(1e-12, bbox["north"] - bbox["south"])
    lng_fraction = (lng - bbox["west"]) / max(1e-12, bbox["east"] - bbox["west"])

    row = int(lat_fraction * bin_rows)
    col = int(lng_fraction * bin_cols)
    row = max(0, min(bin_rows - 1, row))
    col = max(0, min(bin_cols - 1, col))
    return row, col


def _bboxes_intersect(
    a_lat_min: float,
    a_lat_max: float,
    a_lng_min: float,
    a_lng_max: float,
    b_lat_min: float,
    b_lat_max: float,
    b_lng_min: float,
    b_lng_max: float,
) -> bool:
    return not (
        a_lat_max < b_lat_min
        or a_lat_min > b_lat_max
        or a_lng_max < b_lng_min
        or a_lng_min > b_lng_max
    )


def _polygon_edges_intersect_cell(
    polygon: list[tuple[float, float]],
    cell_bounds: tuple[float, float, float, float],
) -> bool:
    lat_min, lat_max, lng_min, lng_max = cell_bounds
    rect_edges = [
        ((lng_min, lat_min), (lng_max, lat_min)),
        ((lng_max, lat_min), (lng_max, lat_max)),
        ((lng_max, lat_max), (lng_min, lat_max)),
        ((lng_min, lat_max), (lng_min, lat_min)),
    ]

    if len(polygon) < 2:
        return False

    for i in range(len(polygon) - 1):
        p1_lat, p1_lng = polygon[i]
        p2_lat, p2_lng = polygon[i + 1]
        seg_p1 = (p1_lng, p1_lat)
        seg_p2 = (p2_lng, p2_lat)
        for rect_p1, rect_p2 in rect_edges:
            if _segments_intersect(seg_p1, seg_p2, rect_p1, rect_p2):
                return True

    return False


def _segments_intersect(
    p1: tuple[float, float],
    p2: tuple[float, float],
    q1: tuple[float, float],
    q2: tuple[float, float],
) -> bool:
    o1 = _orientation(p1, p2, q1)
    o2 = _orientation(p1, p2, q2)
    o3 = _orientation(q1, q2, p1)
    o4 = _orientation(q1, q2, p2)

    if o1 != o2 and o3 != o4:
        return True

    if o1 == 0 and _on_segment(p1, q1, p2):
        return True
    if o2 == 0 and _on_segment(p1, q2, p2):
        return True
    if o3 == 0 and _on_segment(q1, p1, q2):
        return True
    if o4 == 0 and _on_segment(q1, p2, q2):
        return True

    return False


def _orientation(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
) -> int:
    val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
    if abs(val) < 1e-12:
        return 0
    return 1 if val > 0 else 2


def _on_segment(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
) -> bool:
    return (
        min(a[0], c[0]) - 1e-12 <= b[0] <= max(a[0], c[0]) + 1e-12
        and min(a[1], c[1]) - 1e-12 <= b[1] <= max(a[1], c[1]) + 1e-12
    )


def _neighbor_offsets(movement: int) -> list[tuple[int, int, float]]:
    orthogonal = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
    ]
    if movement == 4:
        return orthogonal

    diagonal = [
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    ]
    return orthogonal + diagonal


def _heuristic(node: tuple[int, int], goal: tuple[int, int], movement: int) -> float:
    dx = abs(node[0] - goal[0])
    dy = abs(node[1] - goal[1])

    if movement == 4:
        return float(dx + dy)

    return float(max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy))


def _reconstruct_path(
    came_from: dict[tuple[int, int], tuple[int, int]],
    current: tuple[int, int],
) -> list[tuple[int, int]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def _polyline_distance_m(points: Iterable[tuple[float, float]]) -> float:
    points_list = list(points)
    if len(points_list) < 2:
        return 0.0

    total = 0.0
    for i in range(len(points_list) - 1):
        total += _haversine_m(points_list[i], points_list[i + 1])
    return total


def _haversine_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    lat1, lon1 = a
    lat2, lon2 = b

    r = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    h = (
        math.sin(dphi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    )
    return 2 * r * math.asin(math.sqrt(h))
