package objectgeometry

import (
	"context"
	"fmt"
	"math"
	"sort"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	vision "go.viam.com/rdk/services/vision"
	vis "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/classification"
	objdet "go.viam.com/rdk/vision/objectdetection"
	"go.viam.com/rdk/vision/viscapture"
)

var (
	ShapeFit         = resource.NewModel("viam", "object-geometry", "shape-fit")
	errUnimplemented = errors.New("unimplemented")
	errNoCamera      = errors.New(`DoCommand needs a "camera" set in the module config`)
)

const (
	defaultZMinMM = 95  // fallback band when a cloud is empty
	defaultZMaxMM = 135 // just above the rim's top edge

	defaultMinRadiusMM = 70   // default expected vessel radius range;
	defaultMaxRadiusMM = 200  // override per application via config
	maxFitRMSMM        = 15.0 // above this rms the points don't lie on a circle

	// minContentsHeightMM: a cell must rise at least this far above the floor to
	// count as contents. Below it, the cell is sensor noise. With a baseline the
	// empty floor reads within ~1mm, so this cleanly separates noise from real
	// contents and keeps an empty vessel reading empty.
	minContentsHeightMM = 5

	// Derived rim band shape. The band needs only enough thickness to catch
	// the full rim ring through sensor noise, vessel tilt, and the lip's
	// physical height — NOT to reach toward the interior. A deep band scoops
	// contents-surface points into the circle fit and destroys it.
	zBandBinMM      = 5  // histogram bin for locating the rim surface
	zBandBelowMM    = 12 // band reach below the rim bin
	zBandAboveMM    = 10 // band reach above the rim bin
	minTopBinPoints = 50 // bins with fewer points are strays, not a surface

	minBandPoints = 50 // too few points in the Z band to fit anything
)

func init() {
	resource.RegisterService(vision.API, ShapeFit,
		resource.Registration[vision.Service, *Config]{
			Constructor: newObjectGeometryShapeFit,
		},
	)
}

// Config for the shape-fit vision service.
type Config struct {
	// CameraFrame is the reference frame the segmenter's points arrive in;
	// clouds are transformed from it into the world frame.
	CameraFrame string `json:"camera_frame"`
	// Camera is the camera the segmenter should read from. The vision API
	// takes a camera name per request, so this is only needed for DoCommand.
	Camera    string   `json:"camera,omitempty"`
	Segmenter string   `json:"segmenter"`
	Shapes    []string `json:"shapes,omitempty"`

	// Z band the rim is expected to fall in, world frame. Setting both PINS the
	// band to these absolute values. Leaving them unset (the default) derives
	// the band per detection from the object's own top, so it adapts to the
	// vessel's height instead of a fixed scene height. Both must be set together.
	ZMinMM *float64 `json:"z_min_mm,omitempty"`
	ZMaxMM *float64 `json:"z_max_mm,omitempty"`

	// Expected vessel radius range; fits outside it are rejected. Size these
	// to the application — a pan and a large bin need different bounds.
	MinRadiusMM float64 `json:"min_radius_mm,omitempty"`
	MaxRadiusMM float64 `json:"max_radius_mm,omitempty"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.CameraFrame == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "camera_frame")
	}
	if cfg.Segmenter == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "segmenter")
	}
	if (cfg.ZMinMM == nil) != (cfg.ZMaxMM == nil) {
		return nil, nil, fmt.Errorf("%s: z_min_mm and z_max_mm must be set together", path)
	}
	if cfg.ZMinMM != nil && *cfg.ZMinMM >= *cfg.ZMaxMM {
		return nil, nil, fmt.Errorf("%s: z_min_mm must be less than z_max_mm", path)
	}
	deps := []string{cfg.Segmenter, framesystem.PublicServiceName.String()}
	if cfg.Camera != "" {
		deps = append(deps, cfg.Camera) // image + intrinsics for the analysis overlay
	}
	return deps, nil, nil
}

type objectGeometryShapeFit struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger

	cameraFrame string
	camera      string
	shapes      []string

	zMinMM      float64
	zMaxMM      float64
	fixedBand   bool // true when z_min_mm/z_max_mm pin the band
	minRadiusMM float64
	maxRadiusMM float64

	segmenter vision.Service
	fsService framesystem.Service
	cam       camera.Camera // configured camera; image + intrinsics for the overlay

	mu        sync.Mutex
	baselines []vesselBaseline // vessels captured while empty, via capture_baseline

	overlayMu      sync.Mutex
	overlayVessels []overlayVessel // cached analysis drawn by CaptureAllFromCamera
	overlayAt      time.Time       // last overlay refresh
}

func newObjectGeometryShapeFit(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (vision.Service, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}
	return NewShapeFit(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewShapeFit(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (vision.Service, error) {
	seg, err := vision.FromProvider(deps, conf.Segmenter)
	if err != nil {
		return nil, fmt.Errorf("failed to get segmenter %q: %w", conf.Segmenter, err)
	}

	fsSvc, err := framesystem.FromProvider(deps)
	if err != nil {
		return nil, fmt.Errorf("failed to get framesystem service: %w", err)
	}

	s := &objectGeometryShapeFit{
		name:        name,
		logger:      logger,
		cameraFrame: conf.CameraFrame,
		camera:      conf.Camera,
		shapes:      conf.Shapes,
		zMinMM:      defaultZMinMM,
		zMaxMM:      defaultZMaxMM,
		minRadiusMM: orDefault(conf.MinRadiusMM, defaultMinRadiusMM),
		maxRadiusMM: orDefault(conf.MaxRadiusMM, defaultMaxRadiusMM),
		segmenter:   seg,
		fsService:   fsSvc,
	}
	if conf.ZMinMM != nil {
		s.zMinMM, s.zMaxMM = *conf.ZMinMM, *conf.ZMaxMM
		s.fixedBand = true
	}
	if conf.Camera != "" {
		if cam, err := camera.FromProvider(deps, conf.Camera); err == nil {
			s.cam = cam
		} else {
			logger.Warnf("camera %q unavailable; analysis overlay disabled: %v", conf.Camera, err)
		}
	}
	return s, nil
}

func (s *objectGeometryShapeFit) Name() resource.Name {
	return s.name
}

func (s *objectGeometryShapeFit) Close(context.Context) error {
	return nil
}

// ──────────────────────────────────────────────────────────────────────────
// Vision service API
// ──────────────────────────────────────────────────────────────────────────

func (s *objectGeometryShapeFit) DetectionsFromCamera(ctx context.Context, cameraName string, extra map[string]interface{}) ([]objdet.Detection, error) {
	return nil, errUnimplemented
}

func (s *objectGeometryShapeFit) Detections(ctx context.Context, img *camera.NamedImage, extra map[string]interface{}) ([]objdet.Detection, error) {
	return nil, errUnimplemented
}

func (s *objectGeometryShapeFit) ClassificationsFromCamera(ctx context.Context, cameraName string, n int, extra map[string]interface{}) (classification.Classifications, error) {
	return nil, errUnimplemented
}

func (s *objectGeometryShapeFit) Classifications(ctx context.Context, img *camera.NamedImage, n int, extra map[string]interface{}) (classification.Classifications, error) {
	return nil, errUnimplemented
}

func (s *objectGeometryShapeFit) GetObjectPointClouds(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*vis.Object, error) {
	results, err := s.detect(ctx, cameraName)
	if err != nil {
		return nil, err
	}

	var objects []*vis.Object
	for _, r := range results {
		obj, err := vis.NewObject(r.Cloud)
		if err != nil {
			s.logger.Errorf("failed to create object: %v", err)
			continue
		}
		objects = append(objects, obj)
	}
	return objects, nil
}

func (s *objectGeometryShapeFit) GetProperties(ctx context.Context, extra map[string]interface{}) (*vision.Properties, error) {
	return &vision.Properties{
		ObjectPCDsSupported: true,
	}, nil
}

// CaptureAllFromCamera returns the camera frame with the analysis drawn on it:
// the fitted vessel circle, the eight sector wedges, and each sector's
// coverage value. The app's vision test card polls this to show a live
// visualization of what the analysis is reporting.
func (s *objectGeometryShapeFit) CaptureAllFromCamera(ctx context.Context, cameraName string, opts viscapture.CaptureOptions, extra map[string]interface{}) (viscapture.VisCapture, error) {
	capt := viscapture.VisCapture{}
	if opts.ReturnImage {
		rawImg, err := s.annotatedFrame(ctx, extra)
		if err != nil {
			return capt, err
		}
		namedImg, err := camera.NamedImageFromImage(rawImg, "", "", data.Annotations{})
		if err != nil {
			return capt, err
		}
		capt.Image = &namedImg
	}
	if opts.ReturnObject {
		objs, err := s.GetObjectPointClouds(ctx, cameraName, extra)
		if err != nil {
			return capt, err
		}
		capt.Objects = objs
	}
	return capt, nil
}

// ──────────────────────────────────────────────────────────────────────────
// DoCommand — detect and analyze_region
// ──────────────────────────────────────────────────────────────────────────

func (s *objectGeometryShapeFit) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["detect"]; ok {
		return s.doDetect(ctx)
	}

	if params, ok := cmd["analyze_region"]; ok {
		return s.doAnalyzeRegion(ctx, params)
	}

	if _, ok := cmd["capture_baseline"]; ok {
		return s.doCaptureBaseline(ctx)
	}

	return nil, fmt.Errorf("unknown command, supported: detect, analyze_region, capture_baseline")
}

func (s *objectGeometryShapeFit) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{"status": "ok"}, nil
}

// ──────────────────────────────────────────────────────────────────────────
// Point cloud helper
// ──────────────────────────────────────────────────────────────────────────

// getWorldClouds returns one world-frame cloud per segmented object.
func (s *objectGeometryShapeFit) getWorldClouds(ctx context.Context, cameraName string) ([]pointcloud.PointCloud, error) {
	objects, err := s.segmenter.GetObjectPointClouds(ctx, cameraName, nil)
	if err != nil {
		return nil, fmt.Errorf("segmenter point clouds: %w", err)
	}
	clouds := make([]pointcloud.PointCloud, 0, len(objects))
	for _, obj := range objects {
		transformed, err := s.fsService.TransformPointCloud(ctx, obj, s.cameraFrame, referenceframe.World)
		if err != nil {
			return nil, fmt.Errorf("transform to world: %w", err)
		}
		clouds = append(clouds, transformed)
	}
	return clouds, nil
}

// getWorldCloud merges every segmented object into one world-frame cloud.
// analyze_region wants this: the vessel and its contents are separate
// objects, and the analysis needs both together.
func (s *objectGeometryShapeFit) getWorldCloud(ctx context.Context, cameraName string) (pointcloud.PointCloud, error) {
	clouds, err := s.getWorldClouds(ctx, cameraName)
	if err != nil {
		return nil, err
	}
	merged := pointcloud.NewBasicEmpty()
	for _, c := range clouds {
		var setErr error
		c.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
			setErr = merged.Set(p, d)
			return setErr == nil
		})
		if setErr != nil {
			return nil, fmt.Errorf("merge world cloud: %w", setErr)
		}
	}
	return merged, nil
}

// ──────────────────────────────────────────────────────────────────────────
// detect — fit shapes to the scene
// ──────────────────────────────────────────────────────────────────────────

func orDefault(v, def float64) float64 {
	if v == 0 {
		return def
	}
	return v
}

type shapeResult struct {
	Shape    string
	CenterX  float64
	CenterY  float64
	CenterZ  float64
	Radius   float64
	Length   float64
	Width    float64
	AxisDeg  float64
	RMS      float64
	PointCnt int
	Cloud    pointcloud.PointCloud
}

func (s *objectGeometryShapeFit) detect(ctx context.Context, cameraName string) ([]shapeResult, error) {
	clouds, err := s.getWorldClouds(ctx, cameraName)
	if err != nil {
		return nil, err
	}

	var results []shapeResult
	for i, pcWorld := range clouds {
		if r, ok := s.fitCloud(pcWorld, i); ok {
			results = append(results, r)
		}
	}
	return dedupeOverlapping(results), nil
}

// fitCloud tries to fit a vessel to one object cloud. Each candidate rim band
// (highest surface first) is band-filtered and circle-fit until one passes the
// vessel checks: a tall non-vessel object leaking into the crop (a utensil
// above the rim) claims the highest band, fails the fit, and the next
// candidate — the actual rim — succeeds.
func (s *objectGeometryShapeFit) fitCloud(pcWorld pointcloud.PointCloud, i int) (shapeResult, bool) {
	if len(s.shapes) == 0 {
		return shapeResult{}, false
	}
	for _, band := range s.rimBandCandidates(pcWorld) {
		var pts []r3.Vector
		var zSum float64
		pcWorld.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
			if p.Z >= band[0] && p.Z <= band[1] {
				pts = append(pts, p)
				zSum += p.Z
			}
			return true
		})
		if len(pts) < minBandPoints {
			s.logger.Debugf("object %d band [%.0f, %.0f]: only %d points, trying next surface",
				i, band[0], band[1], len(pts))
			continue
		}

		for _, shape := range s.shapes {
			switch shape {
			case "circle":
				centerX, centerY, r, rms := fitCircleKasa(pts)
				rimZ := zSum / float64(len(pts))

				// Points that don't lie on a circle: this surface is not a
				// rim (a utensil, a sliver). Try the next one down.
				if rms > maxFitRMSMM {
					s.logger.Debugf("object %d band [%.0f, %.0f]: not a circle (rms %.1fmm), trying next surface",
						i, band[0], band[1], rms)
					continue
				}
				// Smaller than twice the rim inset means no usable interior —
				// degenerate junk (a tool tip, a marker dot), not a vessel.
				if r < 2*insetMM {
					s.logger.Debugf("object %d band [%.0f, %.0f]: degenerate fit (radius %.1fmm), trying next surface",
						i, band[0], band[1], r)
					continue
				}
				// A clean circle outside the configured size range is a
				// plausible real object being dropped — warn so a mis-sized
				// min/max_radius_mm config is visible instead of silently
				// detecting nothing.
				if r < s.minRadiusMM || r > s.maxRadiusMM {
					s.logger.Warnf("object %d: circle fit rejected by size bounds (radius %.1fmm, configured range %.0f-%.0fmm)",
						i, r, s.minRadiusMM, s.maxRadiusMM)
					continue
				}

				return shapeResult{
					Shape:    "circle",
					CenterX:  centerX,
					CenterY:  centerY,
					CenterZ:  rimZ,
					Radius:   r,
					RMS:      rms,
					PointCnt: len(pts),
					Cloud:    pcWorld,
				}, true
			}
		}
	}
	return shapeResult{}, false
}

// rimBandCandidates returns candidate world-Z windows to look for the rim in,
// best first. When the band is pinned in config there is exactly one. Derived,
// the candidates are the object's distinct substantial surfaces from highest
// down: histogram the object's Z, keep bins holding enough points to be real
// structure, and collapse adjacent bins into one surface anchored at its top.
// The rim is the vessel's highest structure, so it is usually the first
// candidate — but a taller object leaking into the crop can outrank it, which
// is why callers try candidates in order rather than trusting the first.
func (s *objectGeometryShapeFit) rimBandCandidates(pcWorld pointcloud.PointCloud) [][2]float64 {
	if s.fixedBand {
		return [][2]float64{{s.zMinMM, s.zMaxMM}}
	}
	hist := map[int]int{}
	pcWorld.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		hist[int(math.Floor(p.Z/zBandBinMM))]++
		return true
	})
	var bins []int
	for bin, n := range hist {
		if n >= minTopBinPoints {
			bins = append(bins, bin)
		}
	}
	if len(bins) == 0 {
		return [][2]float64{{s.zMinMM, s.zMaxMM}} // nothing substantial; defaults
	}
	sort.Sort(sort.Reverse(sort.IntSlice(bins)))

	const maxCandidates = 4
	var out [][2]float64
	prev := bins[0] + 2 // sentinel: not adjacent to the first bin
	for _, bin := range bins {
		if bin == prev-1 {
			prev = bin // same contiguous surface; keep its top anchor
			continue
		}
		prev = bin
		topZ := (float64(bin) + 0.5) * zBandBinMM
		out = append(out, [2]float64{topZ - zBandBelowMM, topZ + zBandAboveMM})
		if len(out) == maxCandidates {
			break
		}
	}
	return out
}

// dedupeOverlapping collapses fits of the same physical object. The segmenter
// can return one vessel as several overlapping clouds, each yielding its own
// circle centered inside the others. Keep the strongest fit (most points) in
// each overlapping cluster and drop the rest. Two distinct vessels don't
// overlap, so this never merges separate vessels.
func dedupeOverlapping(results []shapeResult) []shapeResult {
	// Most points first, so the strongest fit in a cluster is kept and the
	// weaker overlapping ones are dropped against it.
	sort.Slice(results, func(i, j int) bool {
		return results[i].PointCnt > results[j].PointCnt
	})

	var kept []shapeResult
	for _, r := range results {
		overlapped := false
		for _, k := range kept {
			// Centers closer than the kept fit's radius means r's center lies
			// inside k's circle: the same object seen twice.
			if math.Hypot(r.CenterX-k.CenterX, r.CenterY-k.CenterY) < k.Radius {
				overlapped = true
				break
			}
		}
		if !overlapped {
			kept = append(kept, r)
		}
	}
	return kept
}

func (s *objectGeometryShapeFit) doDetect(ctx context.Context) (map[string]interface{}, error) {
	if s.camera == "" {
		return nil, errNoCamera
	}
	results, err := s.detect(ctx, s.camera)
	if err != nil {
		return nil, err
	}

	var fitted []map[string]interface{}
	for _, r := range results {
		m := map[string]interface{}{
			"shape": r.Shape,
			"center": map[string]interface{}{
				"x": math.Round(r.CenterX*10) / 10,
				"y": math.Round(r.CenterY*10) / 10,
				"z": math.Round(r.CenterZ*10) / 10,
			},
			"rms_mm":    math.Round(r.RMS*10) / 10,
			"point_cnt": r.PointCnt,
		}
		if r.Shape == "circle" {
			m["radius_mm"] = math.Round(r.Radius*10) / 10
			m["rim_height_mm"] = math.Round(r.CenterZ*10) / 10
		}
		fitted = append(fitted, m)
	}

	return map[string]interface{}{
		"fitted": fitted,
	}, nil
}

// ──────────────────────────────────────────────────────────────────────────
// analyze_region
// ──────────────────────────────────────────────────────────────────────────

const (
	gridCellMM   = 3.0
	minBlobCells = 10
	numSectors   = 8
	insetMM      = 10.0
)

func (s *objectGeometryShapeFit) doAnalyzeRegion(ctx context.Context, params interface{}) (map[string]interface{}, error) {
	p, ok := params.(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("analyze_region: expected object with center, radius_mm")
	}

	centerMap, _ := p["center"].(map[string]interface{})
	centerX, _ := toFloat(centerMap["x"])
	centerY, _ := toFloat(centerMap["y"])
	rimZ, _ := toFloat(centerMap["z"])
	radius, _ := toFloat(p["radius_mm"])

	if radius <= 0 {
		return nil, fmt.Errorf("analyze_region: radius_mm must be > 0")
	}

	if s.camera == "" {
		return nil, errNoCamera
	}
	pcWorld, err := s.getWorldCloud(ctx, s.camera)
	if err != nil {
		return nil, err
	}

	baseline := s.pickBaseline(centerX, centerY)
	result := analyzeRegion(pcWorld, centerX, centerY, rimZ, radius, minContentsHeightMM, baseline)
	if baseline != nil {
		result["floor_source"] = "baseline"
	} else {
		result["floor_source"] = "estimated"
	}
	return result, nil
}

// doCaptureBaseline detects the vessels currently in view — call it while they
// are empty — and stores them as the baseline, replacing any previous capture.
// It returns the captured vessels: center, radius, rim and floor height, and
// cell count.
func (s *objectGeometryShapeFit) doCaptureBaseline(ctx context.Context) (map[string]interface{}, error) {
	if s.camera == "" {
		return nil, errNoCamera
	}
	results, err := s.detect(ctx, s.camera)
	if err != nil {
		return nil, err
	}

	var baselines []vesselBaseline
	vessels := []map[string]interface{}{}
	for _, r := range results {
		if r.Shape != "circle" {
			continue
		}
		g := buildHeightGrid(r.Cloud, r.CenterX, r.CenterY, r.CenterZ, r.Radius)
		if g.occupied == 0 {
			s.logger.Warnf("baseline: vessel at (%.0f, %.0f) has no interior points, skipping",
				r.CenterX, r.CenterY)
			continue
		}
		floorGrid, floorLevel := g.floorSnapshot()
		baselines = append(baselines, vesselBaseline{
			centerX:    r.CenterX,
			centerY:    r.CenterY,
			radius:     r.Radius,
			floorLevel: floorLevel,
			floorGrid:  floorGrid,
			halfCells:  g.halfCells,
			gridCells:  g.occupied,
		})
		vessels = append(vessels, map[string]interface{}{
			"center_mm": map[string]interface{}{
				"x": math.Round(r.CenterX),
				"y": math.Round(r.CenterY),
				"z": 0.0,
			},
			"radius_mm":  math.Round(r.Radius),
			"rim_z_mm":   math.Round(r.CenterZ),
			"floor_z_mm": math.Round(floorLevel),
			"grid_cells": g.occupied,
		})
	}

	s.mu.Lock()
	s.baselines = baselines
	s.mu.Unlock()

	return map[string]interface{}{"vessels": vessels}, nil
}

// pickBaseline returns the stored baseline covering a region, or nil. A
// baseline matches when its center is the nearest one within its own radius of
// the query point.
func (s *objectGeometryShapeFit) pickBaseline(centerX, centerY float64) *vesselBaseline {
	s.mu.Lock()
	defer s.mu.Unlock()

	var best *vesselBaseline
	bestDist := math.Inf(1)
	for i := range s.baselines {
		b := &s.baselines[i]
		if d := math.Hypot(b.centerX-centerX, b.centerY-centerY); d < bestDist {
			bestDist, best = d, b
		}
	}
	if best == nil || bestDist > best.radius {
		return nil // no captured vessel covers this region
	}

	cp := *best // copy so the caller never touches the slice under lock
	return &cp
}

// heightGrid buckets a vessel's interior into a square grid of mean heights.
// It is the shared front-end of analyzeRegion and baseline capture: same
// inset, cell size, and in-circle/below-rim filter.
type heightGrid struct {
	meanZ     [][]float64 // per-cell mean Z; 0 where empty, check cnt
	cnt       [][]int     // points per cell
	halfCells int         // center to edge in cells; also the center index
	occupied  int         // cells holding at least one point
}

func buildHeightGrid(pc pointcloud.PointCloud, centerX, centerY, rimZ, radius float64) heightGrid {
	innerR := radius - insetMM

	// A degenerate fit (tiny, NaN, or Inf radius) yields innerR <= 0, which
	// would make gridWidth negative and panic makeslice. Bail with an empty
	// grid; callers treat occupied == 0 / no cells as "nothing to analyze".
	if !(innerR > 0) || math.IsInf(innerR, 0) {
		return heightGrid{}
	}

	// Square grid of 3mm squares boxing in the vessel interior. Indices run
	// 0..gridWidth-1, so halfCells doubles as the center cell: subtract it
	// from an index to get that cell's offset from the middle.
	halfCells := int(math.Ceil(innerR / gridCellMM))
	gridWidth := 2*halfCells + 1

	zSum := make([][]float64, gridWidth)
	cnt := make([][]int, gridWidth)
	for i := range gridWidth {
		zSum[i] = make([]float64, gridWidth)
		cnt[i] = make([]int, gridWidth)
	}

	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		// is this point inside the vessel and below the rim?
		dx, dy := p.X-centerX, p.Y-centerY
		if dx*dx+dy*dy > innerR*innerR || p.Z > rimZ {
			return true
		}
		// find which square this point lands in
		c := int(math.Round((p.X-centerX)/gridCellMM)) + halfCells
		r := int(math.Round((p.Y-centerY)/gridCellMM)) + halfCells
		if c >= 0 && c < gridWidth && r >= 0 && r < gridWidth {
			zSum[r][c] += p.Z
			cnt[r][c]++
		}
		return true
	})

	meanZ := make([][]float64, gridWidth)
	occupied := 0
	for r := range gridWidth {
		meanZ[r] = make([]float64, gridWidth)
		for c := range gridWidth {
			if cnt[r][c] > 0 {
				meanZ[r][c] = zSum[r][c] / float64(cnt[r][c])
				occupied++
			}
		}
	}
	return heightGrid{meanZ: meanZ, cnt: cnt, halfCells: halfCells, occupied: occupied}
}

// floorCenterFrac: cells within this fraction of the radius define the flat
// bottom. A vessel is a bowl — flat bottom in the middle, walls at the edge —
// so the floor level comes from the center, not the whole interior (whose mean
// or even a low percentile is pulled up by the walls).
const floorCenterFrac = 0.5

// floorSnapshot copies the grid as an empty-floor reference: each observed
// cell's mean Z (NaN where nothing was seen), plus a scalar floor level taken
// from the central cells — the flat bottom, which the sensor undersamples
// relative to the brighter angled walls.
func (g heightGrid) floorSnapshot() (grid [][]float64, floor float64) {
	w := len(g.meanZ)
	grid = make([][]float64, w)

	centerRadius := float64(g.halfCells) * floorCenterFrac
	var central, all []float64
	for r := range w {
		grid[r] = make([]float64, w)
		for c := range w {
			if g.cnt[r][c] == 0 {
				grid[r][c] = math.NaN()
				continue
			}
			grid[r][c] = g.meanZ[r][c]
			all = append(all, g.meanZ[r][c])
			if math.Hypot(float64(c-g.halfCells), float64(r-g.halfCells)) <= centerRadius {
				central = append(central, g.meanZ[r][c])
			}
		}
	}

	// Median of the central cells is the flat-bottom height. Fall back to the
	// overall low end if the center happened to see nothing.
	if len(central) > 0 {
		sort.Float64s(central)
		floor = central[len(central)/2]
	} else if len(all) > 0 {
		sort.Float64s(all)
		floor = all[len(all)/10]
	}
	return grid, floor
}

// vesselBaseline is a vessel captured while empty: where it is, how deep it
// is, and a per-cell floor map. Later analysis measures contents height
// against this instead of the lowest currently-visible cell.
type vesselBaseline struct {
	centerX, centerY float64
	radius           float64
	floorLevel       float64
	floorGrid        [][]float64 // per-cell empty floor Z; NaN where unobserved
	halfCells        int
	gridCells        int
}

// floorAt returns a per-cell floor lookup for an analysis grid of the given
// halfCells. Both grids use the same cell size and index from their center
// cell, so a query cell maps into the baseline grid by a center-to-center
// offset — the grids do NOT need the same size. This matters because the
// detected radius jitters a few mm between frames, which shifts the grid size
// by a cell; requiring identical sizes made the per-cell floor flicker in and
// out (and the tilt cancellation with it). Cells outside the baseline's
// coverage fall back to the scalar floor level.
func (b *vesselBaseline) floorAt(halfCells int) func(r, c int) float64 {
	return func(r, c int) float64 {
		br := r - halfCells + b.halfCells
		bc := c - halfCells + b.halfCells
		if br < 0 || br >= len(b.floorGrid) || bc < 0 || bc >= len(b.floorGrid) {
			return b.floorLevel
		}
		if f := b.floorGrid[br][bc]; !math.IsNaN(f) {
			return f
		}
		return b.floorLevel // a cell the baseline never saw
	}
}

// zeroSectors returns a sector-coverage slice of numSectors zeros. It's a
// []interface{} (not [numSectors]float64) so it serializes into the protobuf
// Struct that DoCommand returns.
func zeroSectors() []interface{} {
	s := make([]interface{}, numSectors)
	for i := range s {
		s[i] = 0.0
	}
	return s
}

// blob is one connected region of contents, in world millimeters. It's the shared
// output of blob detection, consumed both by analyze_region (as a map) and by
// the Detections overlay (projected to image pixels).
type blob struct {
	centerX, centerY   float64
	areaMM2            float64
	axisDeg            float64
	majorLen, minorLen float64
	meanHeight         float64
}

// regionGrid builds the height grid and derives the median and per-cell floor
// lookup shared by the analysis. ok is false when the region has too few cells
// to analyze.
func regionGrid(pc pointcloud.PointCloud, centerX, centerY, rimZ, radius float64, baseline *vesselBaseline) (g heightGrid, medZ float64, floorAt func(r, c int) float64, ok bool) {
	g = buildHeightGrid(pc, centerX, centerY, rimZ, radius)

	var all []float64
	for r := range g.meanZ {
		for c := range g.meanZ[r] {
			if g.cnt[r][c] > 0 {
				all = append(all, g.meanZ[r][c])
			}
		}
	}
	if len(all) < 10 {
		return g, 0, nil, false
	}

	// medZ splits the interior into "taller than typical" and "at floor
	// level"; minZ is the fallback floor when no baseline is supplied.
	sort.Float64s(all)
	minZ := all[0]
	medZ = all[len(all)/2]
	floorAt = func(r, c int) float64 { return minZ }
	if baseline != nil {
		floorAt = baseline.floorAt(g.halfCells)
	}
	return g, medZ, floorAt, true
}

// findBlobs flood-fills connected contents cells and describes each with a centroid,
// area, PCA axis, and extent. A cell is contents only if it's above the median and
// rises at least minContentsHeightMM over the floor.
func findBlobs(meanZ [][]float64, cnt [][]int, halfCells int, centerX, centerY, medZ, minContentsHeightMM float64, floorAt func(r, c int) float64) []blob {
	gridWidth := len(meanZ)

	// Occupancy: no tuned threshold — the median splits contents from floor, and
	// the height gate drops sensor noise so an empty vessel stays empty.
	occ := make([][]bool, gridWidth)
	for r := range gridWidth {
		occ[r] = make([]bool, gridWidth)
		for c := range gridWidth {
			occ[r][c] = cnt[r][c] > 0 && meanZ[r][c] > medZ &&
				meanZ[r][c]-floorAt(r, c) >= minContentsHeightMM
		}
	}

	visited := make([][]bool, gridWidth)
	for i := range gridWidth {
		visited[i] = make([]bool, gridWidth)
	}
	// 8-connectivity: diagonal neighbors count, so a region joined only at the
	// corners stays one blob.
	type cell struct{ r, c int }
	dirs := [8][2]int{{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}}

	var blobs []blob
	for r := range gridWidth {
		for c := range gridWidth {
			if !occ[r][c] || visited[r][c] {
				continue
			}
			var comp []cell
			q := []cell{{r, c}}
			visited[r][c] = true
			for len(q) > 0 {
				cur := q[0]
				q = q[1:]
				comp = append(comp, cur)
				for _, d := range dirs {
					nr, nc := cur.r+d[0], cur.c+d[1]
					if nr >= 0 && nr < gridWidth && nc >= 0 && nc < gridWidth &&
						!visited[nr][nc] && occ[nr][nc] {
						visited[nr][nc] = true
						q = append(q, cell{nr, nc})
					}
				}
			}
			if len(comp) < minBlobCells {
				continue
			}

			var bx, by, bh float64
			var blobPts int
			for _, cl := range comp {
				wx := centerX + float64(cl.c-halfCells)*gridCellMM
				wy := centerY + float64(cl.r-halfCells)*gridCellMM
				w := cnt[cl.r][cl.c]
				bx += wx * float64(w)
				by += wy * float64(w)
				bh += (meanZ[cl.r][cl.c] - floorAt(cl.r, cl.c)) * float64(w)
				blobPts += w
			}
			tp := float64(blobPts)
			bx /= tp
			by /= tp
			bh /= tp
			area := float64(len(comp)) * gridCellMM * gridCellMM

			// PCA on the blob's cells: the 2x2 covariance's larger-eigenvalue
			// vector is the major axis (a stretched smear vs. a round mound).
			var cxx, cxy, cyy float64
			for _, cl := range comp {
				ddx := centerX + float64(cl.c-halfCells)*gridCellMM - bx
				ddy := centerY + float64(cl.r-halfCells)*gridCellMM - by
				w := float64(cnt[cl.r][cl.c])
				cxx += ddx * ddx * w
				cxy += ddx * ddy * w
				cyy += ddy * ddy * w
			}
			cxx /= tp
			cxy /= tp
			cyy /= tp
			tr := cxx + cyy
			det := cxx*cyy - cxy*cxy
			disc := math.Sqrt(math.Max(tr*tr/4-det, 0))
			lam1 := tr/2 + disc
			var angle float64
			if math.Abs(cxy) > 1e-9 {
				angle = math.Atan2(lam1-cxx, cxy)
			} else if cyy > cxx {
				angle = math.Pi / 2
			}

			// Extent along each axis from the actual cell spread.
			cosA, sinA := math.Cos(angle), math.Sin(angle)
			var minMaj, maxMaj, minMin, maxMin float64
			for i, cl := range comp {
				ddx := centerX + float64(cl.c-halfCells)*gridCellMM - bx
				ddy := centerY + float64(cl.r-halfCells)*gridCellMM - by
				pMaj := ddx*cosA + ddy*sinA
				pMin := -ddx*sinA + ddy*cosA
				if i == 0 {
					minMaj, maxMaj = pMaj, pMaj
					minMin, maxMin = pMin, pMin
				} else {
					minMaj = math.Min(minMaj, pMaj)
					maxMaj = math.Max(maxMaj, pMaj)
					minMin = math.Min(minMin, pMin)
					maxMin = math.Max(maxMin, pMin)
				}
			}

			blobs = append(blobs, blob{
				centerX:    bx,
				centerY:    by,
				areaMM2:    area,
				axisDeg:    angle * 180 / math.Pi,
				majorLen:   maxMaj - minMaj + gridCellMM,
				minorLen:   maxMin - minMin + gridCellMM,
				meanHeight: bh,
			})
		}
	}
	return blobs
}

// analyzeRegion examines the contents of a vessel. A non-nil baseline supplies
// a per-cell empty-floor reference captured earlier; without one, heights are
// measured from the lowest currently-visible cell (an estimated floor).
func analyzeRegion(pc pointcloud.PointCloud, centerX, centerY, rimZ, radius, minContentsHeightMM float64, baseline *vesselBaseline) map[string]interface{} {
	empty := map[string]interface{}{
		"centroid_mm":      map[string]interface{}{"x": 0.0, "y": 0.0},
		"mean_height_mm":   0.0,
		"sector_fill":      zeroSectors(),
		"sector_height_mm": zeroSectors(),
		"blobs":            []interface{}{},
	}
	g, medZ, floorAt, ok := regionGrid(pc, centerX, centerY, rimZ, radius, baseline)
	if !ok {
		return empty
	}
	meanZ, cnt, halfCells := g.meanZ, g.cnt, g.halfCells
	gridWidth := len(meanZ)

	// Centroid weighted by height above the floor times point count, so it is
	// pulled toward where the contents actually is rather than the vessel center.
	var totalPts int
	var sumH float64
	var wx, wy, wTotal float64
	for r := range gridWidth {
		for c := range gridWidth {
			if cnt[r][c] == 0 {
				continue
			}
			h := meanZ[r][c] - floorAt(r, c) // height above floor
			n := cnt[r][c]                   // how many points landed here

			// Running totals for mean height: point-weighted so densely
			// sampled cells count more than sparse ones.
			totalPts += n
			sumH += h * float64(n)

			// This cell's real-world position (grid index back to mm).
			cellX := centerX + float64(c-halfCells)*gridCellMM
			cellY := centerY + float64(r-halfCells)*gridCellMM

			// Weight = height above floor * point count. Tall, well-sampled
			// cells (where the contents is) dominate; flat or sparse cells
			// contribute ~nothing. wx/wy accumulate position*weight and
			// wTotal the weights, so wx/wTotal below is the center of mass.
			w := h * float64(n)
			wx += cellX * w
			wy += cellY * w
			wTotal += w
		}
	}

	centroid := map[string]interface{}{"x": 0.0, "y": 0.0}
	var meanHeight float64
	if wTotal > 0 {
		centroid = map[string]interface{}{
			"x": math.Round(wx / wTotal),
			"y": math.Round(wy / wTotal),
		}
	}
	if totalPts > 0 {
		meanHeight = math.Round(sumH / float64(totalPts))
	}

	// Per-sector contents summary: split the interior into numSectors equal
	// wedges by angle about the center and report, for each wedge, two
	// absolute quantities that together are unambiguous:
	//
	//	fill      — the fraction of the wedge's observed area that has
	//	            contents on it (cells above the contents-height gate).
	//	            Bare patches lower it directly; an even full wedge is
	//	            ~1.0, an empty wedge 0.
	//	height_mm — the mean depth of the contents where present. A wedge
	//	            that's half bare with a 30mm pile reads fill 0.5,
	//	            height 30 — not a misleading 15mm average.
	//
	// Heights are measured above the per-cell floor, so with a baseline a
	// tilted empty vessel reads zero fill rather than a gradient.
	var secObserved, secContents [numSectors]int
	var secHeightSum [numSectors]float64
	for r := range gridWidth {
		for c := range gridWidth {
			if cnt[r][c] == 0 {
				continue
			}
			dx := float64(c-halfCells) * gridCellMM
			dy := float64(r-halfCells) * gridCellMM
			// Atan2 returns (-pi, pi]; shift to [0, 2pi) so sector indices
			// run counterclockwise from +X with no negative wraparound.
			a := math.Atan2(dy, dx)
			if a < 0 {
				a += 2 * math.Pi
			}
			sec := int(a / (2 * math.Pi) * numSectors)
			if sec >= numSectors {
				sec = numSectors - 1
			}
			secObserved[sec]++
			if h := meanZ[r][c] - floorAt(r, c); h >= minContentsHeightMM {
				secContents[sec]++
				secHeightSum[sec] += h
			}
		}
	}
	// []interface{}, not [numSectors]float64: a fixed-size array can't be
	// serialized into a protobuf Struct for the DoCommand response.
	sectorFill := zeroSectors()
	sectorHeight := zeroSectors()
	for i := range numSectors {
		if secObserved[i] > 0 {
			sectorFill[i] = math.Round(float64(secContents[i])/float64(secObserved[i])*100) / 100
		}
		if secContents[i] > 0 {
			sectorHeight[i] = math.Round(secHeightSum[i] / float64(secContents[i]))
		}
	}

	// Blob detection, shared with the Detections overlay.
	blobs := []interface{}{}
	for _, b := range findBlobs(meanZ, cnt, halfCells, centerX, centerY, medZ, minContentsHeightMM, floorAt) {
		blobs = append(blobs, map[string]interface{}{
			"centroid_mm":    map[string]interface{}{"x": math.Round(b.centerX), "y": math.Round(b.centerY)},
			"area_mm2":       math.Round(b.areaMM2),
			"major_axis_deg": math.Round(b.axisDeg),
			"major_len_mm":   math.Round(b.majorLen),
			"minor_len_mm":   math.Round(b.minorLen),
			"mean_height_mm": math.Round(b.meanHeight),
		})
	}

	return map[string]interface{}{
		"centroid_mm":      centroid,
		"mean_height_mm":   meanHeight,
		"sector_fill":      sectorFill,
		"sector_height_mm": sectorHeight,
		"blobs":            blobs,
	}
}

// ──────────────────────────────────────────────────────────────────────────
// Circle fit (Kasa method)
// ──────────────────────────────────────────────────────────────────────────

// fitCircleKasa fits a circle to the XY projection of pts (Z is ignored) and
// returns the center, radius, and rms residual in the same units as the input.
//
// A circle is (x-centerX)^2 + (y-centerY)^2 = r^2
//
// The tradeoff is bias. Least squares on this form minimizes the algebraic
// residual, and for a point at distance d from the center that residual is
// d^2-r^2 = (d-r)(d+r) ~= 2r*(d-r) — the geometric error weighted by ~2r, so
// points farther from the center count more. That weighting is symmetric on
// a full ring and cancels, but on a partial arc it pulls the solution toward
// a smaller radius. Taubin and Pratt fit the same closed form with a
// normalization that removes this; prefer them if rims start arriving as
// short arcs rather than near-complete rings.
func fitCircleKasa(pts []r3.Vector) (centerX, centerY, r, rms float64) {
	// Accumulate the sums that form the normal equations. Note "z" here is
	// the substituted term x^2+y^2, NOT the Z coordinate.
	var sxx, sxy, syy, sx, sy, sxz, syz, sz float64
	n := float64(len(pts))
	for _, p := range pts {
		z := p.X*p.X + p.Y*p.Y
		sxx += p.X * p.X
		sxy += p.X * p.Y
		syy += p.Y * p.Y
		sx += p.X
		sy += p.Y
		sxz += p.X * z
		syz += p.Y * z
		sz += z
	}

	// Setting the three partial derivatives of sum((z + Dx + Ey + F)^2) to
	// zero gives this system, stored as an augmented matrix [A | b]:
	//
	//	d/dD:  D*Sxx + E*Sxy + F*Sx = -Sxz
	//	d/dE:  D*Sxy + E*Syy + F*Sy = -Syz
	//	d/dF:  D*Sx  + E*Sy  + F*n  = -Sz
	a := [3][4]float64{
		{sxx, sxy, sx, -sxz},
		{sxy, syy, sy, -syz},
		{sx, sy, n, -sz},
	}

	// Gaussian elimination to upper triangular. The pivot search swaps in the
	// row with the largest leading coefficient — without it, a near-zero
	// pivot would blow up the division below.
	for col := 0; col < 3; col++ {
		piv := col
		for row := col + 1; row < 3; row++ {
			if math.Abs(a[row][col]) > math.Abs(a[piv][col]) {
				piv = row
			}
		}
		a[col], a[piv] = a[piv], a[col]
		for row := col + 1; row < 3; row++ {
			f := a[row][col] / a[col][col]
			for c := col; c < 4; c++ {
				a[row][c] -= f * a[col][c]
			}
		}
	}

	// Back-substitution: solve the last row for one unknown, then work
	// upward substituting the unknowns already recovered.
	sol := [3]float64{}
	for row := 2; row >= 0; row-- {
		v := a[row][3]
		for c := row + 1; c < 3; c++ {
			v -= a[row][c] * sol[c]
		}
		sol[row] = v / a[row][row]
	}

	// Undo the change of variables: D = -2cx, E = -2cy, F = centerX^2+centerY^2-r^2.
	d, e, f := sol[0], sol[1], sol[2]
	centerX, centerY = -d/2, -e/2
	r = math.Sqrt(centerX*centerX + centerY*centerY - f)

	// Residual is the *geometric* distance from each point to the fitted
	// circle, not the algebraic quantity minimized above. That makes rms a
	// real distance in input units, so it can be compared against a physical
	// tolerance to judge whether the points are actually circular.
	var sq float64
	for _, p := range pts {
		dist := math.Hypot(p.X-centerX, p.Y-centerY) - r
		sq += dist * dist
	}
	rms = math.Sqrt(sq / n)
	return centerX, centerY, r, rms
}

// toFloat converts interface{} to float64 (handles json number types).
func toFloat(v interface{}) (float64, bool) {
	switch n := v.(type) {
	case float64:
		return n, true
	case int:
		return float64(n), true
	case int64:
		return float64(n), true
	default:
		return 0, false
	}
}
