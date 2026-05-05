let map;
let geocoder;

const state = {
  selecting: "start",
  start: null,
  end: null,
  startMarker: null,
  endMarker: null,
  pathLine: null,
  obstaclePolygons: [],
  bboxRectangle: null,
};

function initApp() {
  map = new google.maps.Map(document.getElementById("map"), {
    center: { lat: 36.8065, lng: 10.1815 },
    zoom: 13,
    mapTypeControl: true,
    fullscreenControl: true,
    streetViewControl: false,
  });

  geocoder = new google.maps.Geocoder();

  map.addListener("click", (event) => {
    const latLng = event.latLng;
    if (!latLng) {
      return;
    }

    const point = { lat: latLng.lat(), lng: latLng.lng() };
    if (state.selecting === "start") {
      setStart(point);
      setStatus("Start set from map click.");
      state.selecting = "end";
    } else {
      setEnd(point);
      setStatus("End set from map click.");
      state.selecting = "start";
    }
  });

  wireUi();
  setStatus("Ready. Choose start and end (click map or geocode address).");
}

function wireUi() {
  document.getElementById("setStartBtn").addEventListener("click", () => {
    state.selecting = "start";
    setStatus("Click on map to set START point.");
  });

  document.getElementById("setEndBtn").addEventListener("click", () => {
    state.selecting = "end";
    setStatus("Click on map to set END point.");
  });

  document.getElementById("geocodeStartBtn").addEventListener("click", async () => {
    const value = document.getElementById("startInput").value.trim();
    if (!value) {
      setStatus("Enter a start address first.");
      return;
    }
    await geocodeAndSet(value, "start");
  });

  document.getElementById("geocodeEndBtn").addEventListener("click", async () => {
    const value = document.getElementById("endInput").value.trim();
    if (!value) {
      setStatus("Enter an end address first.");
      return;
    }
    await geocodeAndSet(value, "end");
  });

  document.getElementById("findPathBtn").addEventListener("click", findPath);
  document.getElementById("clearBtn").addEventListener("click", clearPathOnly);

  attachAutocomplete("startInput", "start");
  attachAutocomplete("endInput", "end");
}

function attachAutocomplete(inputId, targetType) {
  const input = document.getElementById(inputId);
  const autocomplete = new google.maps.places.Autocomplete(input, {
    fields: ["formatted_address", "geometry", "name"],
  });

  autocomplete.addListener("place_changed", () => {
    const place = autocomplete.getPlace();
    if (!place.geometry || !place.geometry.location) {
      setStatus("Selected place has no coordinates.");
      return;
    }

    const point = {
      lat: place.geometry.location.lat(),
      lng: place.geometry.location.lng(),
    };

    if (targetType === "start") {
      setStart(point);
    } else {
      setEnd(point);
    }

    if (place.formatted_address) {
      input.value = place.formatted_address;
    }

    map.panTo(point);
    map.setZoom(15);
    setStatus(`${targetType.toUpperCase()} set from autocomplete.`);
  });
}

function geocodeAndSet(address, targetType) {
  return geocoder
    .geocode({ address })
    .then((res) => {
      if (!res.results || !res.results.length) {
        setStatus(`No geocode result for ${targetType}.`);
        return;
      }

      const result = res.results[0];
      const loc = result.geometry.location;
      const point = { lat: loc.lat(), lng: loc.lng() };

      if (targetType === "start") {
        setStart(point);
      } else {
        setEnd(point);
      }

      map.panTo(point);
      map.setZoom(15);
      setStatus(`${targetType.toUpperCase()} set from geocoding.`);
    })
    .catch(() => {
      setStatus(`Geocoding failed for ${targetType}.`);
    });
}

function setStart(point) {
  state.start = point;
  if (!state.startMarker) {
    state.startMarker = new google.maps.Marker({
      map,
      title: "Start",
      icon: markerSymbol("#22c55e"),
    });
  }
  state.startMarker.setPosition(point);
  document.getElementById("startCoords").textContent = `Start: ${fmt(point.lat)}, ${fmt(point.lng)}`;
}

function setEnd(point) {
  state.end = point;
  if (!state.endMarker) {
    state.endMarker = new google.maps.Marker({
      map,
      title: "End",
      icon: markerSymbol("#ef4444"),
    });
  }
  state.endMarker.setPosition(point);
  document.getElementById("endCoords").textContent = `End: ${fmt(point.lat)}, ${fmt(point.lng)}`;
}

function markerSymbol(color) {
  return {
    path: google.maps.SymbolPath.CIRCLE,
    fillColor: color,
    fillOpacity: 1,
    strokeColor: "#ffffff",
    strokeWeight: 1.2,
    scale: 7,
  };
}

function clearPathOnly() {
  if (state.pathLine) {
    state.pathLine.setMap(null);
    state.pathLine = null;
  }

  if (state.bboxRectangle) {
    state.bboxRectangle.setMap(null);
    state.bboxRectangle = null;
  }

  for (const polygon of state.obstaclePolygons) {
    polygon.setMap(null);
  }
  state.obstaclePolygons = [];

  setStatus("Path and overlays cleared.");
  resetResults();
}

function resetResults() {
  updateResult("rFound", "-");
  updateResult("rCost", "-");
  updateResult("rDistance", "-");
  updateResult("rExplored", "-");
  updateResult("rSteps", "-");
  updateResult("rTime", "-");
  updateResult("rBlocked", "-");
  updateResult("rDifficult", "-");
}

async function findPath() {
  if (!state.start || !state.end) {
    setStatus("Set both start and end before running A*.");
    return;
  }

  clearPathOnly();

  const rows = clampInt(document.getElementById("rowsInput").value, 25, 180, 100);
  const cols = clampInt(document.getElementById("colsInput").value, 25, 180, 100);
  const movement = clampInt(document.getElementById("movementSelect").value, 4, 8, 8);
  const difficultCost = clampFloat(document.getElementById("difficultCostInput").value, 1, 15, 3);

  const payload = {
    start: state.start,
    end: state.end,
    rows,
    cols,
    movement,
    difficult_cost: difficultCost,
  };

  setStatus("Running A* with real obstacle extraction...");

  try {
    const controller = new AbortController();
    const timeoutMs = 45000;
    const timeoutHandle = setTimeout(() => controller.abort(), timeoutMs);

    const response = await fetch("/api/pathfind", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
      signal: controller.signal,
    });
    clearTimeout(timeoutHandle);

    const data = await response.json();

    if (!response.ok) {
      throw new Error(data.error || "Pathfinding failed.");
    }
    if (data.diagnostics) {
      console.log("Pathfinding diagnostics:", data.diagnostics);
    }

    drawBbox(data.bbox);
    drawObstacles(data.obstacles);

    if (data.path_found && Array.isArray(data.path) && data.path.length > 1) {
      drawPath(data.path);
      fitPath(data.path);
      setStatus("Path found and rendered.");
    } else {
      setStatus("No valid path found in this sampled area/grid.");
    }

    if (data.obstacle_warning) {
      setStatus(data.obstacle_warning);
    } else if (data.obstacle_source === "fallback_empty") {
      setStatus("Path found, but obstacle fetch failed (fallback mode). Try again for real obstacles.");
    }

    updateResult("rFound", data.path_found ? "Yes" : "No");
    updateResult("rCost", String(data.cost));
    updateResult("rDistance", String(data.distance_m));
    updateResult("rExplored", String(data.explored_count));
    updateResult("rSteps", String(data.path_steps));
    updateResult("rTime", String(data.compute_time_ms));
    updateResult("rBlocked", String(data.grid.blocked_cells));
    updateResult("rDifficult", String(data.grid.difficult_cells));
  } catch (error) {
    if (error && error.name === "AbortError") {
      setStatus("Request timed out after 45s. Reduce grid size or use shorter/segmented route.");
      return;
    }
    setStatus(error.message || "Unexpected error.");
  }
}

function drawPath(path) {
  state.pathLine = new google.maps.Polyline({
    map,
    path,
    geodesic: true,
    strokeColor: "#1d4ed8",
    strokeOpacity: 0.95,
    strokeWeight: 4,
  });
}

function drawBbox(bbox) {
  state.bboxRectangle = new google.maps.Rectangle({
    map,
    bounds: {
      north: bbox.north,
      south: bbox.south,
      east: bbox.east,
      west: bbox.west,
    },
    strokeColor: "#0f766e",
    strokeOpacity: 0.65,
    strokeWeight: 1,
    fillOpacity: 0,
  });
}

function drawObstacles(obstacles) {
  renderPolygonSet(obstacles.buildings || [], {
    fillColor: "#fb7185",
    fillOpacity: 0.34,
    strokeColor: "#be123c",
  }, 300);

  renderPolygonSet(obstacles.water || [], {
    fillColor: "#60a5fa",
    fillOpacity: 0.32,
    strokeColor: "#1d4ed8",
  }, 300);

  renderPolygonSet(obstacles.difficult || [], {
    fillColor: "#eab308",
    fillOpacity: 0.22,
    strokeColor: "#b45309",
  }, 300);
}

function renderPolygonSet(polygons, style, limit) {
  const count = Math.min(limit, polygons.length);
  for (let i = 0; i < count; i += 1) {
    const ring = polygons[i];
    if (!Array.isArray(ring) || ring.length < 3) {
      continue;
    }

    const path = ring
      .filter((point) => Array.isArray(point) && point.length === 2)
      .map(([lat, lng]) => ({ lat, lng }));

    if (path.length < 3) {
      continue;
    }

    const polygon = new google.maps.Polygon({
      map,
      paths: path,
      strokeColor: style.strokeColor,
      strokeOpacity: 0.65,
      strokeWeight: 1,
      fillColor: style.fillColor,
      fillOpacity: style.fillOpacity,
      clickable: false,
    });

    state.obstaclePolygons.push(polygon);
  }
}

function fitPath(path) {
  const bounds = new google.maps.LatLngBounds();
  path.forEach((point) => bounds.extend(point));
  map.fitBounds(bounds, 45);
}

function updateResult(id, value) {
  document.getElementById(id).textContent = value;
}

function setStatus(message) {
  document.getElementById("status").textContent = `Status: ${message}`;
}

function fmt(v) {
  return Number(v).toFixed(6);
}

function clampInt(value, min, max, fallback) {
  const parsed = Number.parseInt(value, 10);
  if (Number.isNaN(parsed)) {
    return fallback;
  }
  return Math.max(min, Math.min(max, parsed));
}

function clampFloat(value, min, max, fallback) {
  const parsed = Number.parseFloat(value);
  if (Number.isNaN(parsed)) {
    return fallback;
  }
  return Math.max(min, Math.min(max, parsed));
}
