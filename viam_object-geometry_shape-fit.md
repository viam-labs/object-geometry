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

**Rim band, pinned vs derived.** By default the band is derived per detection
from the object's distinct substantial surfaces (5mm height-bins holding
enough points to be real structure), tried from the highest down: each
candidate surface gets a thin band around it (~12mm below, ~10mm above — just
enough for noise, tilt, and the lip) and a circle fit, and the first surface
that fits like a vessel wins. The rim is the vessel's highest structure, so
it's normally the first candidate; a taller object leaking into the crop (a
utensil beside the vessel) fails the fit and falls through to the rim.
Anchoring on surfaces rather than a percentile of all points keeps the band
on the rim even when a full vessel's contents dominate the point count.
Pinning with `z_min_mm`/`z_max_mm` remains the escape hatch on sensors where
the derivation misbehaves. Known limit: contents heaped *above* the rim
across the vessel are indistinguishable from the vessel top — pin the band if
that's a real operating condition.

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

`CaptureAllFromCamera` returns the camera frame with the analysis drawn on it:
the fitted circle, the eight sector wedges shaded green by fill, each wedge
labeled `fill% depth`, and the overall mean contents height (`h=`) at the
center. The Viam app's vision test card polls this, giving a live
visualization; detection + analysis are cached and refreshed at most every few
seconds so the stream stays responsive. Requires `camera` in the config and a
camera that reports intrinsics.

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
  "sector_fill": [0.06, 0.15, 0.6, 0.94, 0.99, 0.96, 0.61, 0.22],
  "sector_height_mm": [8, 6, 10, 17, 19, 15, 11, 8],
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
| `sector_fill` | Eight wedges counterclockwise from +X: the fraction of each wedge's observed area that has contents on it (cells at least 5mm above the floor). Absolute, not rescaled — an empty wedge is 0, an evenly covered one ~1.0, a half-bare one ~0.5. |
| `sector_height_mm` | The mean contents depth per wedge, measured only where contents are present. A wedge that's half bare with a 30mm pile reads fill 0.5, height 30 — not a misleading 15mm average. |
| `blobs` | Connected regions taller than the vessel's median height and at least 5mm above the floor (an internal noise gate). `major_axis_deg` is the elongation direction from PCA; the lengths are the region's actual footprint. |

Reading the output: fill answers "where is material and how completely does it
cover?", height answers "how deep is it where it is", and `mean_height_mm`
remains the overall "is there anything in it?" signal. These are honest only
with a captured baseline (`floor_source: "baseline"`): the estimated-floor
fallback measures from the lowest visible cell, which inflates fill on a
tilted or unevenly-sensed vessel. The internal height gate keeps an empty
vessel from producing noise blobs; regions under 10 grid cells are also
dropped.
