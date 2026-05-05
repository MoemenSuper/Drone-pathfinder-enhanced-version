from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

import requests

OVERPASS_ENDPOINTS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
]
REQUEST_TIMEOUT_SECONDS = 15
MAX_RETRIES_PER_ENDPOINT = 1


@dataclass
class FeaturePolygons:
    buildings: list[list[tuple[float, float]]]
    water: list[list[tuple[float, float]]]
    difficult: list[list[tuple[float, float]]]


def fetch_feature_polygons(bbox: dict[str, float]) -> FeaturePolygons:
    query = _build_overpass_query(bbox)
    payload = _request_overpass_json(query)

    buildings: list[list[tuple[float, float]]] = []
    water: list[list[tuple[float, float]]] = []
    difficult: list[list[tuple[float, float]]] = []

    for element in payload.get("elements", []):
        tags = element.get("tags", {})
        polygons = _extract_polygons(element)
        if not polygons:
            continue

        if _is_building(tags):
            buildings.extend(polygons)
        elif _is_water(tags):
            water.extend(polygons)
        elif _is_difficult(tags):
            difficult.extend(polygons)

    return FeaturePolygons(
        buildings=_deduplicate_polygons(buildings),
        water=_deduplicate_polygons(water),
        difficult=_deduplicate_polygons(difficult),
    )


def empty_features() -> FeaturePolygons:
    return FeaturePolygons(buildings=[], water=[], difficult=[])


def point_in_polygon(lat: float, lng: float, polygon: list[tuple[float, float]]) -> bool:
    if len(polygon) < 3:
        return False

    inside = False
    j = len(polygon) - 1

    for i, (yi, xi) in enumerate(polygon):
        yj, xj = polygon[j]

        intersects = ((yi > lat) != (yj > lat)) and (
            lng < (xj - xi) * (lat - yi) / ((yj - yi) or 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i

    return inside


def _request_overpass_json(query: str) -> dict[str, Any]:
    errors: list[str] = []
    headers = {
        "User-Agent": "EnhancedDronePathfinder/1.0",
        "Accept": "application/json",
    }

    for endpoint in OVERPASS_ENDPOINTS:
        for attempt in range(1, MAX_RETRIES_PER_ENDPOINT + 1):
            try:
                response = requests.post(
                    endpoint,
                    data={"data": query},
                    headers=headers,
                    timeout=REQUEST_TIMEOUT_SECONDS,
                )

                if response.status_code >= 400:
                    body_preview = (response.text or "").strip().replace("\n", " ")[:140]
                    raise requests.HTTPError(
                        f"HTTP {response.status_code} from {endpoint}"
                        + (f" [{body_preview}]" if body_preview else "")
                    )

                try:
                    return response.json()
                except ValueError as exc:
                    raise RuntimeError(f"Invalid JSON from {endpoint}.") from exc

            except Exception as exc:  # noqa: BLE001
                errors.append(f"{endpoint} attempt {attempt}: {exc}")
                if attempt < MAX_RETRIES_PER_ENDPOINT:
                    time.sleep(0.7 * attempt)

    detail = " | ".join(errors[:4])
    raise RuntimeError(
        "Failed to query obstacle data from Overpass API mirrors. "
        f"Details: {detail}"
    )


def _build_overpass_query(bbox: dict[str, float]) -> str:
    south = bbox["south"]
    west = bbox["west"]
    north = bbox["north"]
    east = bbox["east"]

    return f"""
[out:json][timeout:40];
(
  way["building"]({south},{west},{north},{east});
  relation["building"]({south},{west},{north},{east});
  way["building:part"]({south},{west},{north},{east});
  relation["building:part"]({south},{west},{north},{east});

  way["natural"="water"]({south},{west},{north},{east});
  relation["natural"="water"]({south},{west},{north},{east});
  way["landuse"="reservoir"]({south},{west},{north},{east});
  relation["landuse"="reservoir"]({south},{west},{north},{east});
  way["landuse"="basin"]({south},{west},{north},{east});
  relation["landuse"="basin"]({south},{west},{north},{east});

  way["landuse"="forest"]({south},{west},{north},{east});
  relation["landuse"="forest"]({south},{west},{north},{east});
  way["natural"="wood"]({south},{west},{north},{east});
  relation["natural"="wood"]({south},{west},{north},{east});
);
out geom;
""".strip()


def _extract_polygons(element: dict[str, Any]) -> list[list[tuple[float, float]]]:
    polygons: list[list[tuple[float, float]]] = []

    if element.get("type") == "way":
        coords = _geometry_to_polygon(element.get("geometry", []))
        if coords:
            polygons.append(coords)
        return polygons

    if element.get("type") == "relation":
        for member in element.get("members", []):
            if member.get("type") != "way":
                continue
            role = member.get("role", "")
            if role and role not in {"outer"}:
                continue
            coords = _geometry_to_polygon(member.get("geometry", []))
            if coords:
                polygons.append(coords)

    return polygons


def _geometry_to_polygon(geometry: list[dict[str, Any]]) -> list[tuple[float, float]]:
    if len(geometry) < 3:
        return []

    polygon: list[tuple[float, float]] = []
    for point in geometry:
        lat = point.get("lat")
        lon = point.get("lon")
        if lat is None or lon is None:
            return []
        polygon.append((float(lat), float(lon)))

    # Only closed ways should be interpreted as polygons.
    if not _is_closed_ring(polygon):
        return []

    return polygon


def _is_closed_ring(polygon: list[tuple[float, float]], tol: float = 1e-9) -> bool:
    if len(polygon) < 4:
        return False
    lat0, lng0 = polygon[0]
    lat1, lng1 = polygon[-1]
    return abs(lat0 - lat1) <= tol and abs(lng0 - lng1) <= tol


def _is_building(tags: dict[str, Any]) -> bool:
    return "building" in tags or "building:part" in tags


def _is_water(tags: dict[str, Any]) -> bool:
    if tags.get("natural") == "water":
        return True
    if tags.get("waterway") == "riverbank":
        return True
    if tags.get("landuse") in {"reservoir", "basin"}:
        return True
    return False


def _is_difficult(tags: dict[str, Any]) -> bool:
    if tags.get("landuse") == "forest":
        return True
    if tags.get("natural") == "wood":
        return True
    return False


def _deduplicate_polygons(polygons: list[list[tuple[float, float]]]) -> list[list[tuple[float, float]]]:
    seen: set[str] = set()
    deduped: list[list[tuple[float, float]]] = []

    for polygon in polygons:
        key = "|".join(f"{lat:.6f},{lng:.6f}" for lat, lng in polygon)
        if key in seen:
            continue
        seen.add(key)
        deduped.append(polygon)

    return deduped
