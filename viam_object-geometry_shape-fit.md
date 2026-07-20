# Model viam:object-geometry:shape-fit

A vision service that fits circles to segmented objects and analyzes the
contents of the vessel it finds.

Wraps another vision service (the segmenter) and the frame system. The
segmenter isolates objects; this model transforms them into the world frame,
filters to the rim height band, and fits a circle to each object's XY
projection. Fitting runs per object, so a scene with two vessels yields two
results rather than one circle averaged across both.

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
  "max_radius_mm": <float>,
  "max_fit_rms_mm": <float>
}
```

### Attributes

| Name             | Type     | Inclusion | Default | Description |
|------------------|----------|-----------|---------|-------------|
| `camera_frame`   | string   | Required  | —       | Reference frame the segmenter's points arrive in. Clouds are transformed from here into the world frame. This is a frame name, not a camera resource name. |
| `segmenter`      | string   | Required  | —       | Name of the vision service that segments objects. |
| `camera`         | string   | Optional  | —       | Camera the segmenter reads from. Only used by `DoCommand`; the vision API takes a camera name per request. `DoCommand` errors without it. |
| `shapes`         | []string | Optional  | `[]`    | Shapes to fit. Only `"circle"` is implemented. Empty means no fitting. |
| `z_min_mm`       | float    | Optional  | `95`    | Bottom of the rim height band, world frame. Must be set together with `z_max_mm`. |
| `z_max_mm`       | float    | Optional  | `135`   | Top of the rim height band, world frame. Must be greater than `z_min_mm`. |
| `min_radius_mm`  | float    | Optional  | `70`    | Below this, the fit is flagged `suspect`. |
| `max_radius_mm`  | float    | Optional  | `200`   | Above this, the fit is flagged `suspect`. |
| `max_fit_rms_mm` | float    | Optional  | `15`    | Above this residual, the fit is flagged `suspect`. |

The radius and rms bounds do **not** reject a fit — an out-of-range result is
still returned with `suspect: true` and a human-readable `warning`.

The default Z band is calibrated for a specific stove and pan: above the
stove's top surface, just past the rim's top edge. Set it explicitly for other
hardware.

### Example Configuration

```json
{
  "camera_frame": "cam",
  "camera": "pan-crop",
  "segmenter": "pan-segmenter",
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

`center.z` and `rim_height_mm` are both the mean height of the banded points.
A fit outside the configured bounds gains two extra fields:

```json
{
  "suspect": true,
  "warning": "fit doesn't look like a pan (radius 41.2mm, expected 70-200mm; rms 4.1mm, expected under 15mm)"
}
```

`fitted` is absent when nothing matched.

### analyze_region

Analyzes the contents of a vessel, given its center and radius — typically fed
straight from a `detect` result. Unlike `detect`, this merges every segmented
object into one cloud, because the vessel and its contents come back as
separate objects and the analysis needs them together.

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
exclude rim points.

Response:

```json
{
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
| `centroid_mm` | Height-weighted center of mass, pulled toward where material is piled. |
| `mean_height_mm` | Average height above the vessel floor. `0` means empty or perfectly flat. |
| `sector_coverage` | Eight wedges counterclockwise from +X, rescaled so `0` is the flattest wedge and `1` the tallest. Shows *where* material sits, not how much. |
| `blobs` | Connected regions taller than the vessel's median height. `major_axis_deg` is the elongation direction from PCA; the lengths are the region's actual footprint. |

The analysis uses no absolute height thresholds — the floor is the lowest grid
cell and "material" is anything above the median — so results are comparable
across vessels and fill levels.

Two consequences of that. `sector_coverage` is rescaled per call, so an evenly
filled vessel and an empty one both read as all zeros; check `mean_height_mm`
to distinguish them. And blob detection is thresholded on the median, so an
empty vessel can still produce small noise blobs; regions under 10 grid cells
are dropped.

Both commands error if `camera` is unset in the config. `analyze_region`
additionally requires `radius_mm` to be positive.
