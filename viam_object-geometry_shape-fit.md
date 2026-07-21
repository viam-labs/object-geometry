# Model viam:object-geometry:shape-fit

A vision service that fits circles to segmented objects and analyzes the
contents of the vessel it finds.

Wraps another vision service (the segmenter) and the frame system. The
segmenter isolates objects; this model transforms them into the world frame,
filters to the rim height band, and fits a circle to each object's XY
projection. Fitting runs per object.

All positions are in millimeters in the world frame.

## Configuration

```json
{
  "camera_frame": <string>,
  "segmenter": <string>,
  "camera": <string>,
  "shapes": [<string>],
  "z_min_mm": <float>,
  "z_max_mm": <float>,
  "min_radius_mm": <float>,
  "max_radius_mm": <float>
}
```

### Attributes

| Name            | Type     | Inclusion | Default | Description |
|-----------------|----------|-----------|---------|-------------|
| `camera_frame`  | string   | Required  | —       | Reference frame the segmenter's points arrive in. Clouds are transformed from here into the world frame. This is a frame name, not a camera resource name. |
| `segmenter`     | string   | Required  | —       | Name of the vision service that segments objects. |
| `camera`        | string   | Optional  | —       | Camera the segmenter reads from. Only used by `DoCommand`; the vision API takes a camera name per request. `DoCommand` errors without it. |
| `shapes`        | []string | Optional  | `[]`    | Shapes to fit. Only `"circle"` is implemented. Empty means no fitting. |
| `z_min_mm`      | float    | Optional  | —       | Setting this together with `z_max_mm` **pins** the rim band to absolute world heights. Left unset, the band is derived per detection from the object's own top. |
| `z_max_mm`      | float    | Optional  | —       | See `z_min_mm`. Must be greater than it; both must be set together. |
| `min_radius_mm` | float    | Optional  | `70`    | Expected vessel radius range; fits outside it are rejected (not returned). Size these to the application — a frying pan and a large bin need different bounds. |
| `max_radius_mm` | float    | Optional  | `200`   | See `min_radius_mm`. |

A fit whose points don't actually lie on a circle (high fit residual) is also
rejected internally, so slivers and partial edges the segmenter returns don't
come back as vessels.

**Rim band, pinned vs derived.** By default the band is derived from each
object's own top (95th percentile of its Z, minus an internal 40mm band
width), so it adapts to wherever the vessel sits with no hard-coded scene
heights. Pinning with `z_min_mm`/`z_max_mm` is the escape hatch when the
derived band misbehaves — e.g. a depth sensor that flattens the vessel
interior close to rim level can drag the derived band (and the reported rim
height) down.

### Example Configuration

```json
{
  "camera_frame": "cam",
  "camera": "crop-cam",
  "segmenter": "my-segmenter",
  "shapes": ["circle"]
}
```

## Vision service API

`GetObjectPointClouds` returns one object per fitted shape, carrying the
source point cloud. The camera name comes from the request, so `camera` in the
config is not consulted.

`GetProperties` reports `object_point_clouds_supported: true`. Detections and
classifications are not implemented and return an error.

## DoCommand

All commands error if `camera` is unset in the config.

### detect

Fits shapes to whatever the segmenter returns from the configured `camera`.

```json
{ "detect": true }
```

Response:

```json
{
  "fitted": [
    {
      "shape": "circle",
      "center": { "x": 412.3, "y": -88.1, "z": 118.4 },
      "radius_mm": 121.7,
      "rim_height_mm": 118.4,
      "rms_mm": 4.2,
      "point_cnt": 3184
    }
  ]
}
```

`center.z` and `rim_height_mm` are both the mean height of the banded rim
points. Fits outside the radius/rms bounds are dropped, and overlapping fits
of the same object are deduplicated, so every entry is a distinct vessel.
`fitted` is absent when nothing matched.

### capture_baseline

Captures the vessels currently in view — run it **while the vessels are
empty** — and stores each one's per-cell floor map. Later `analyze_region`
calls measure contents height against this captured floor instead of the
lowest currently-visible cell, which matters once contents cover the floor.
Each capture replaces the previous one, and baselines live in memory: they
are lost when the module restarts.

```json
{ "capture_baseline": true }
```

Response:

```json
{
  "vessels": [
    {
      "center_mm": { "x": 412, "y": -88, "z": 0 },
      "radius_mm": 122,
      "rim_z_mm": 118,
      "floor_z_mm": 67,
      "grid_cells": 4470
    }
  ]
}
```

| Field | Meaning |
|-------|---------|
| `center_mm` | Vessel center. `z` is always 0; the rim height is `rim_z_mm`. |
| `rim_z_mm` | World height of the rim. |
| `floor_z_mm` | Floor level: the median height of the central cells (the flat bottom), not the mean of the whole bowl-shaped interior. Note that depth sensors often under-read a dark concave interior, so this can sit above the true floor. |
| `grid_cells` | Interior cells that received points — a coverage sanity check. |

### analyze_region

Analyzes the contents of a vessel, given its center and radius — typically fed
straight from a `detect` result. Merges every segmented object into one cloud
before analysis.

```json
{
  "analyze_region": {
    "center": { "x": 412.3, "y": -88.1, "z": 118.4 },
    "radius_mm": 121.7
  }
}
```

`center.z` is the rim height; points above it are ignored, so the analysis
sees only what's inside. The radius is shrunk by 10mm before analysis to
exclude rim points, and `radius_mm` must be positive. If a captured baseline
covers this center, its per-cell floor is used; otherwise the floor is
estimated as the lowest visible cell.

Response:

```json
{
  "floor_source": "baseline",
  "centroid_mm": { "x": 420, "y": -75 },
  "mean_height_mm": 12,
  "sector_coverage": [0.2, 0.5, 1.0, 0.8, 0.3, 0.1, 0, 0.1],
  "blobs": [
    {
      "centroid_mm": { "x": 445, "y": -60 },
      "area_mm2": 2187,
      "major_axis_deg": 37,
      "major_len_mm": 84,
      "minor_len_mm": 39,
      "mean_height_mm": 21
    }
  ]
}
```

| Field | Meaning |
|-------|---------|
| `floor_source` | `"baseline"` when a captured floor was used, `"estimated"` otherwise. |
| `centroid_mm` | Height-weighted center of mass, pulled toward where material is piled. |
| `mean_height_mm` | Average height above the floor. `~0` means empty or perfectly flat. |
| `sector_coverage` | Eight wedges counterclockwise from +X, rescaled so `0` is the flattest wedge and `1` the tallest. Shows *where* material sits, not how much. |
| `blobs` | Connected regions taller than the vessel's median height and at least 5mm above the floor (an internal noise gate). `major_axis_deg` is the elongation direction from PCA; the lengths are the region's actual footprint. |

Reading the output: `sector_coverage` is rescaled per call, so an evenly
filled vessel and an empty one both span 0..1 or read all zeros — it cannot
tell empty from uneven on its own. Gate any consumer on `mean_height_mm`
first ("is there anything in it?"), then use `sector_coverage` for "where is
it?". The internal height gate keeps an empty vessel from producing noise
blobs; regions under 10 grid cells are also dropped.
