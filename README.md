# Enhanced Drone Pathfinder

A sophisticated web-based drone pathfinding application that combines the A* algorithm with real-world obstacle extraction from OpenStreetMap to calculate optimal flight routes while avoiding buildings, water, and difficult terrain.

## 🚀 Features

- **Real-World Obstacle Detection**: Automatically extracts buildings, water bodies, and forests from OpenStreetMap using the Overpass API
- **Intelligent A* Pathfinding**: Implements A* algorithm with support for both 4-directional and 8-directional movement modes
- **Adaptive Grid Resolution**: Automatically adjusts grid density based on route distance for optimal performance
- **Smart Retry Logic**: Intelligently retries failed pathfinding attempts with adjusted parameters for dense urban areas
- **Long-Range Optimization**: Special handling for routes exceeding 30km for improved performance
- **Interactive Google Maps Integration**: Set start/end points via map clicks or address geocoding
- **Comprehensive Diagnostics**: Detailed performance metrics and pathfinding diagnostics
- **Real-Time Visualization**: View obstacles, bounding boxes, and computed paths on an interactive map
- **Terrain Cost Weighting**: Adjust cost multipliers for difficult terrain (forests, etc.)

## 📋 Technology Stack

### Backend
- **Python 3.x** with Flask framework
- **A* Pathfinding Algorithm** with spatial polygon indexing
- **Overpass API** for OpenStreetMap data retrieval
- **Haversine formula** for accurate distance calculations

### Frontend
- **Google Maps API** for map rendering and geocoding
- **Vanilla JavaScript** for client-side logic
- **Modern CSS3** with responsive design and dark theme

## 🛠️ Installation

### Prerequisites
- Python 3.8+
- pip (Python package manager)
- Google Maps API Key
- Internet connection (for Overpass API and Google Maps)

### Setup Steps

1. **Clone or download the project**
   ```bash
   cd enhanced_google_maps_pathfinder
   ```

2. **Create a virtual environment (recommended)**
   ```bash
   python -m venv venv
   ```

   On Windows:
   ```bash
   venv\Scripts\activate
   ```

   On macOS/Linux:
   ```bash
   source venv/bin/activate
   ```

3. **Install dependencies**
   ```bash
   pip install flask requests
   ```

**Set up Google Maps API Key (Optional)**
   - The application includes a hard-coded demo API key for testing
   - To use your own key, obtain one from [Google Cloud Console](https://console.cloud.google.com/) and set the environment variable:
     ```bash
     set GOOGLE_MAPS_API_KEY=your_api_key_here
     ```
     Or on macOS/Linux:
     ```bash
     export GOOGLE_MAPS_API_KEY=your_api_key_here
     ```

5. **Run the application**
   ```bash
   python app.py
   ```
   
   The application will be available at `http://localhost:5000`

## 📖 Usage

### Setting Start and End Points

**Option 1: Click on Map**
1. Click "Set Start" button and then click a location on the map
2. Click "Set End" button and then click a location on the map

**Option 2: Address Geocoding**
1. Type an address in the "Start Address" field and click "GO"
2. Type an address in the "End Address" field and click "GO"

**Option 3: Google Places Autocomplete**
1. Start typing in the address field and select from suggestions

### Configuring A* Parameters

- **Movement Mode**: Choose between 4-directional (orthogonal) or 8-directional (with diagonals)
- **Grid Rows/Cols**: Adjust grid resolution (25-180). Higher values = finer resolution but slower computation
- **Difficult Terrain Cost**: Set cost multiplier for difficult terrain like forests (1.0-15.0)

### Running the Pathfinding

1. Set both start and end points
2. Adjust A* parameters as needed
3. Click "FIND PATH" button
4. View results in the "Flight Report" section
5. Observe the path visualization on the map

### Understanding the Results

- **Path Found**: Whether a valid path was discovered
- **Total Cost**: Accumulated cost along the path (considering terrain difficulty)
- **Distance (m)**: Total distance in meters
- **Explored Nodes**: Number of grid cells evaluated by A*
- **Path Steps**: Number of waypoints in the path
- **Compute Time (ms)**: Pathfinding computation duration
- **Blocked Cells**: Number of cells blocked by obstacles
- **Difficult Cells**: Number of cells with difficult terrain

## 🏗️ Project Structure

```
enhanced_google_maps_pathfinder/
├── app.py                 # Flask application entry point
├── pathfinding.py         # A* algorithm implementation
├── osm_obstacles.py       # OpenStreetMap data fetching and parsing
├── templates/
│   └── index.html        # Main UI template
├── static/
│   ├── app.js            # Client-side JavaScript logic
│   └── styles.css        # UI styling
└── README.md             # This file
```

## 📚 Core Modules

### `app.py`
Main Flask application containing:
- Route handlers (`/`, `/api/health`, `/api/pathfind`)
- Parameter validation and processing
- Integration of obstacle extraction and A* pathfinding
- Intelligent retry strategy for failed routes
- Long-range route optimization
- Response formatting with comprehensive diagnostics

### `pathfinding.py`
A* pathfinding engine featuring:
- `GeoGrid`: Represents the geographic grid with walkable cells and costs
- `run_astar()`: Implements A* algorithm with heuristic-based exploration
- `build_geo_grid()`: Creates grid from geographic bounds and obstacles
- Spatial polygon indexing for efficient obstacle collision detection
- Support for both 4 and 8-directional movement

### `osm_obstacles.py`
OpenStreetMap integration including:
- `fetch_feature_polygons()`: Queries Overpass API for obstacles
- Multi-endpoint fallback for reliability
- Polygon extraction and deduplication
- Support for multiple Overpass mirrors
- Building, water, and forest detection

## 🎛️ API Reference

### POST `/api/pathfind`

Computes an optimized path between two points.

**Request Body:**
```json
{
  "start": {"lat": 36.8065, "lng": 10.1815},
  "end": {"lat": 36.8165, "lng": 10.1915},
  "rows": 100,
  "cols": 100,
  "movement": 8,
  "difficult_cost": 3.0
}
```

**Response:**
```json
{
  "path": [{"lat": 36.8065, "lng": 10.1815}, ...],
  "path_found": true,
  "cost": 125.5,
  "distance_m": 850.25,
  "explored_count": 450,
  "path_steps": 42,
  "compute_time_ms": 245.5,
  "obstacle_source": "overpass",
  "obstacle_warning": null,
  "grid": {
    "rows": 100,
    "cols": 100,
    "blocked_cells": 340,
    "difficult_cells": 120,
    "start_cell": {"row": 45, "col": 50},
    "end_cell": {"row": 55, "col": 60},
    "start_used": {"lat": 36.8065, "lng": 10.1815},
    "end_used": {"lat": 36.8165, "lng": 10.1915}
  },
  "diagnostics": { /* detailed diagnostics */ }
}
```

### GET `/api/health`

Health check endpoint.

**Response:**
```json
{"ok": true}
```

## ⚙️ Configuration

### Environment Variables

- `GOOGLE_MAPS_API_KEY`: Your Google Maps API key (defaults to a demo key)
- `DRONE_LOG_LEVEL`: Logging level (default: INFO)

### Route Distance Thresholds

The application automatically adjusts parameters based on route distance:

| Distance Range | Bbox Padding | Min Span Degrees | Grid Limit | Obstacle Source |
|---|---|---|---|---|
| ≤ 250m | 0.12 | 0.0018 | Full grid | Overpass |
| 250-800m | 0.16 | 0.0024 | Full grid | Overpass |
| 800-2km | 0.22 | 0.0035 | Full grid | Overpass |
| 2-6km | 0.28 | 0.0060 | Full grid | Overpass |
| 6-30km | 0.35 | 0.01 | Full grid | Overpass |
| > 30km | 0.06 | 0.01 | Max 80×80 | Simplified |

## 🐛 Troubleshooting

### Path Not Found
- Increase grid resolution (rows/cols) for better accuracy
- Check if obstacles are over-blocking (water bodies can sometimes cause this)
- Try a different route or add intermediate waypoints

### Timeout Error
- Reduce grid size (rows/cols) to speed up computation
- Use intermediate waypoints for very long routes
- For routes > 30km, the system automatically uses simplified obstacle extraction

### Geocoding Failed
- Ensure the address is valid and complete
- Check that Google Maps API key has geocoding enabled
- Verify internet connection

### Obstacle Extraction Failed
- Application will fall back to using no obstacles
- Try again - Overpass API can be temporarily unavailable
- Very large bounding boxes might hit API limits

## 🗺️ Map Legend

- 🟢 **Green Marker**: Start point
- 🔴 **Red Marker**: End point
- 🔵 **Blue Line**: Computed A* path
- 🟫 **Brown Polygons**: Buildings (blocked)
- 🔵 **Cyan Polygons**: Water bodies (blocked)
- 🟩 **Green Polygons**: Forests (difficult terrain)
- ⬜ **Light Gray Rectangle**: Bounding box for pathfinding

## 📊 Performance Characteristics

- **Typical computation time**: 50-500ms for urban routes up to 10km
- **Max grid size**: 180×180 cells (32,400 cells)
- **Grid cell size**: Varies by route distance and grid resolution
- **Obstacle extraction**: 1-3 seconds for typical urban areas
- **Long-range penalty**: Applied automatically for routes > 30km

## 📝 License

This project is provided as-is for educational and research purposes.

## 🤝 Contributing

Contributions are welcome! Please feel free to:
- Report bugs
- Suggest improvements
- Add new features
- Improve documentation

## 📧 Contact & Support

For issues, questions, or feedback, please refer to the project documentation or create an issue in the repository.
This is purely a academical project.
It Simulates google maps.

---

**Disclaimer**: This tool is for educational and research purposes. Always validate pathfinding results and consult official flight regulations and maps before conducting actual drone operations.
