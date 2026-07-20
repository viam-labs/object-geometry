package objectgeometry

import (
	"context"
	"fmt"
	"image"
	"math"
	"sort"
	"sync"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"

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

// Defaults mirror the pan-detect client's constants.
const (
	defaultZMinMM      = 95  // above the portable stove's top surface
	defaultZMaxMM      = 135 // just above the rim's top edge
	defaultMinRadiusMM = 70  // plausible pan radius range
	defaultMaxRadiusMM = 200 //
	defaultMaxFitRMSMM = 15  // above this the fit doesn't look like a pan

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

	// Z band the rim is expected to fall in, world frame. Defaults match the
	// pan-detect client: above the stove's top surface, just past the rim's
	// top edge. Both must be set together to override.
	ZMinMM *float64 `json:"z_min_mm,omitempty"`
	ZMaxMM *float64 `json:"z_max_mm,omitempty"`

	// Fit acceptance bounds. A fit outside these is still returned, flagged
	// "suspect" — the client warns and carries on rather than dropping it.
	MinRadiusMM float64 `json:"min_radius_mm,omitempty"`
	MaxRadiusMM float64 `json:"max_radius_mm,omitempty"`
	MaxFitRMSMM float64 `json:"max_fit_rms_mm,omitempty"`
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
	minRadiusMM float64
	maxRadiusMM float64
	maxFitRMSMM float64

	segmenter vision.Service
	fsService framesystem.Service

	mu        sync.Mutex
	baselines []vesselBaseline // empty vessels captured at recipe start
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
		maxFitRMSMM: orDefault(conf.MaxFitRMSMM, defaultMaxFitRMSMM),
		segmenter:   seg,
		fsService:   fsSvc,
	}
	if conf.ZMinMM != nil {
		s.zMinMM, s.zMaxMM = *conf.ZMinMM, *conf.ZMaxMM
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

func (s *objectGeometryShapeFit) Detections(ctx context.Context, img image.Image, extra map[string]interface{}) ([]objdet.Detection, error) {
	return nil, errUnimplemented
}

func (s *objectGeometryShapeFit) ClassificationsFromCamera(ctx context.Context, cameraName string, n int, extra map[string]interface{}) (classification.Classifications, error) {
	return nil, errUnimplemented
}

func (s *objectGeometryShapeFit) Classifications(ctx context.Context, img image.Image, n int, extra map[string]interface{}) (classification.Classifications, error) {
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

func (s *objectGeometryShapeFit) CaptureAllFromCamera(ctx context.Context, cameraName string, captureOptions viscapture.CaptureOptions, extra map[string]interface{}) (viscapture.VisCapture, error) {
	return viscapture.VisCapture{}, errUnimplemented
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
	// Warning is set when the fit is outside the expected bounds. The result
	// is still returned, matching the client's warn-and-continue behavior.
	Warning string
}

func (s *objectGeometryShapeFit) detect(ctx context.Context, cameraName string) ([]shapeResult, error) {
	clouds, err := s.getWorldClouds(ctx, cameraName)
	if err != nil {
		return nil, err
	}

	var results []shapeResult
	for i, pcWorld := range clouds {
		// Keep only points in the rim height band
		var pts []r3.Vector
		var zSum float64
		pcWorld.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
			if p.Z >= s.zMinMM && p.Z <= s.zMaxMM {
				pts = append(pts, p)
				zSum += p.Z
			}
			return true
		})

		if len(pts) < minBandPoints {
			s.logger.Warnf("object %d: only %d points in z band [%.0f, %.0f]mm, skipping",
				i, len(pts), s.zMinMM, s.zMaxMM)
			continue
		}

		for _, shape := range s.shapes {
			switch shape {
			case "circle":
				centerX, centerY, r, rms := fitCircleKasa(pts)
				rimZ := zSum / float64(len(pts))

				// Out-of-range fits are still returned, flagged suspect: the
				// client warns and keeps going rather than dropping them.
				var warning string
				if r < s.minRadiusMM || r > s.maxRadiusMM || rms > s.maxFitRMSMM {
					warning = fmt.Sprintf(
						"fit doesn't look like a pan (radius %.1fmm, expected %.0f-%.0fmm; rms %.1fmm, expected under %.0fmm)",
						r, s.minRadiusMM, s.maxRadiusMM, rms, s.maxFitRMSMM)
					s.logger.Warnf("object %d: %s", i, warning)
				}

				results = append(results, shapeResult{
					Shape:    "circle",
					CenterX:  centerX,
					CenterY:  centerY,
					CenterZ:  rimZ,
					Radius:   r,
					RMS:      rms,
					PointCnt: len(pts),
					Cloud:    pcWorld,
					Warning:  warning,
				})
			}
		}
	}

	return results, nil
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
		if r.Warning != "" {
			m["suspect"] = true
			m["warning"] = r.Warning
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
	result := analyzeRegion(pcWorld, centerX, centerY, rimZ, radius, baseline)
	if baseline != nil {
		result["floor_source"] = "baseline"
	} else {
		result["floor_source"] = "estimated"
	}
	return result, nil
}

// doCaptureBaseline detects the empty vessels currently in view and stores them
// as the recipe-start baseline, replacing any previous capture. It returns the
// captured vessels: center, radius, rim and floor height, and cell count.
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
		floorGrid, floorMean := g.floorSnapshot()
		baselines = append(baselines, vesselBaseline{
			centerX:   r.CenterX,
			centerY:   r.CenterY,
			radius:    r.Radius,
			floorMean: floorMean,
			floorGrid: floorGrid,
			halfCells: g.halfCells,
			gridCells: g.occupied,
		})
		vessels = append(vessels, map[string]interface{}{
			"center_mm": map[string]interface{}{
				"x": math.Round(r.CenterX),
				"y": math.Round(r.CenterY),
				"z": 0.0,
			},
			"radius_mm":       math.Round(r.Radius),
			"rim_z_mm":        math.Round(r.CenterZ),
			"floor_z_mm_mean": math.Round(floorMean),
			"grid_cells":      g.occupied,
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

// floorSnapshot copies the grid as an empty-floor reference: each observed
// cell's mean Z, NaN where nothing was seen, plus the mean of observed cells.
func (g heightGrid) floorSnapshot() (grid [][]float64, mean float64) {
	w := len(g.meanZ)
	grid = make([][]float64, w)
	var sum float64
	var n int
	for r := range w {
		grid[r] = make([]float64, w)
		for c := range w {
			if g.cnt[r][c] > 0 {
				grid[r][c] = g.meanZ[r][c]
				sum += g.meanZ[r][c]
				n++
			} else {
				grid[r][c] = math.NaN()
			}
		}
	}
	if n > 0 {
		mean = sum / float64(n)
	}
	return grid, mean
}

// vesselBaseline is an empty vessel captured at recipe start: where it is, how
// deep it is, and a per-cell floor map. Later analysis measures food height
// against this instead of the lowest currently-visible cell.
type vesselBaseline struct {
	centerX, centerY float64
	radius           float64
	floorMean        float64
	floorGrid        [][]float64 // per-cell empty floor Z; NaN where unobserved
	halfCells        int
	gridCells        int
}

// floorAt returns a per-cell floor lookup aligned to a grid of the given
// halfCells. If the baseline grid doesn't line up — a different radius, so a
// different cell count — it falls back to the scalar mean floor.
func (b *vesselBaseline) floorAt(halfCells int) func(r, c int) float64 {
	if b.halfCells != halfCells {
		return func(r, c int) float64 { return b.floorMean }
	}
	return func(r, c int) float64 {
		if f := b.floorGrid[r][c]; !math.IsNaN(f) {
			return f
		}
		return b.floorMean // a cell the baseline never saw
	}
}

// analyzeRegion examines the contents of a vessel. A non-nil baseline supplies
// a per-cell empty-floor reference captured earlier; without one, heights are
// measured from the lowest currently-visible cell (an estimated floor).
func analyzeRegion(pc pointcloud.PointCloud, centerX, centerY, rimZ, radius float64, baseline *vesselBaseline) map[string]interface{} {
	g := buildHeightGrid(pc, centerX, centerY, rimZ, radius)
	meanZ, cnt, halfCells := g.meanZ, g.cnt, g.halfCells
	gridWidth := len(meanZ)

	var allMeanZs []float64
	for r := range gridWidth {
		for c := range gridWidth {
			if cnt[r][c] > 0 {
				allMeanZs = append(allMeanZs, meanZ[r][c])
			}
		}
	}

	empty := map[string]interface{}{
		"centroid_mm":     map[string]interface{}{"x": 0.0, "y": 0.0},
		"mean_height_mm":  0.0,
		"sector_coverage": [8]float64{},
		"blobs":           []interface{}{},
	}
	if len(allMeanZs) < 10 {
		return empty
	}

	// medZ splits the interior into "taller than typical" and "at floor
	// level"; minZ is the fallback floor when no baseline is supplied.
	sort.Float64s(allMeanZs)
	minZ := allMeanZs[0]
	medZ := allMeanZs[len(allMeanZs)/2]

	// floorAt gives each cell's height reference: the captured empty floor
	// when a baseline is supplied, otherwise minZ.
	floorAt := func(r, c int) float64 { return minZ }
	if baseline != nil {
		floorAt = baseline.floorAt(halfCells)
	}

	// Centroid weighted by height above the floor times point count, so it is
	// pulled toward where the food actually is rather than the vessel center.
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
			// cells (where the food is) dominate; flat or sparse cells
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

	// Sector coverage: split the interior into numSectors equal wedges by
	// angle about the center, average each wedge's cell heights, then rescale
	// the wedge means onto 0..1 where 0 is the flattest wedge and 1 the
	// tallest. Rescaling per call is what makes the output comparable across
	// vessels and fill levels — it reports *where* the food is piled, not how
	// much there is. The consequence is that a perfectly even vessel and a
	// perfectly empty one both read as all-zeros.
	var secSum [numSectors]float64
	var secCnt [numSectors]int
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
			secSum[sec] += meanZ[r][c]
			secCnt[sec]++
		}
	}
	var secMean [numSectors]float64
	secMin, secMax := math.Inf(1), math.Inf(-1)
	for i := range numSectors {
		if secCnt[i] > 0 {
			secMean[i] = secSum[i] / float64(secCnt[i])
			secMin = math.Min(secMin, secMean[i])
			secMax = math.Max(secMax, secMean[i])
		}
	}
	var sectorCoverage [numSectors]float64
	if span := secMax - secMin; span > 0 {
		for i := range numSectors {
			if secCnt[i] > 0 {
				sectorCoverage[i] = math.Round((secMean[i]-secMin)/span*10) / 10
			}
		}
	}

	// Blob detection. Occupancy is "taller than the median cell", which needs
	// no tuned threshold: the median splits the interior into food and floor
	// on its own. The tradeoff is that it always finds *something* — in an
	// empty vessel, half the cells still sit above the median and become
	// blobs made of noise. minBlobCells is what suppresses those.
	occ := make([][]bool, gridWidth)
	for r := range gridWidth {
		occ[r] = make([]bool, gridWidth)
		for c := range gridWidth {
			occ[r][c] = cnt[r][c] > 0 && meanZ[r][c] > medZ
		}
	}

	visited := make([][]bool, gridWidth)
	for i := range gridWidth {
		visited[i] = make([]bool, gridWidth)
	}
	// 8-connectivity: diagonal neighbors count, so a food region joined only
	// at the corners stays one blob instead of splitting in two.
	type cell struct{ r, c int }
	dirs := [8][2]int{{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}}

	var blobs []interface{}
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

			// PCA on the blob's cells to find its elongation and heading.
			// Build the 2x2 covariance of cell offsets from the blob
			// centroid, weighted by point count:
			//
			//	[ cxx  cxy ]
			//	[ cxy  cyy ]
			//
			// Its eigenvectors are the blob's principal axes and the larger
			// eigenvalue's vector is the major axis — the direction the blob
			// is longest along, which is what tells a stretched smear apart
			// from a round mound.
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
			// Closed-form eigenvalues of a symmetric 2x2: the characteristic
			// polynomial lam^2 - tr*lam + det = 0 solves to tr/2 +/- disc.
			// lam1 is the larger, so its eigenvector is the major axis. The
			// Max(..., 0) guards a tiny negative discriminant from roundoff —
			// a symmetric matrix always has real eigenvalues.
			tr := cxx + cyy
			det := cxx*cyy - cxy*cxy
			disc := math.Sqrt(math.Max(tr*tr/4-det, 0))
			lam1 := tr/2 + disc

			// Eigenvector for lam1 is proportional to (cxy, lam1-cxx), giving
			// the major-axis angle. When cxy ~= 0 the axes are already aligned
			// with X and Y, and the eigenvector formula degenerates to 0/0 —
			// so pick the axis with the larger variance directly.
			var angle float64
			if math.Abs(cxy) > 1e-9 {
				angle = math.Atan2(lam1-cxx, cxy)
			} else if cyy > cxx {
				angle = math.Pi / 2
			}

			// Project every cell onto the two axes (a rotation by -angle) and
			// track the extent along each. Lengths come from the actual
			// spread of cells rather than from the eigenvalues, so a blob's
			// reported size is its real footprint, not a Gaussian estimate.
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

			blobs = append(blobs, map[string]interface{}{
				"centroid_mm":    map[string]interface{}{"x": math.Round(bx), "y": math.Round(by)},
				"area_mm2":       math.Round(area),
				"major_axis_deg": math.Round(angle * 180 / math.Pi),
				"major_len_mm":   math.Round(maxMaj - minMaj + gridCellMM),
				"minor_len_mm":   math.Round(maxMin - minMin + gridCellMM),
				"mean_height_mm": math.Round(bh),
			})
		}
	}
	if blobs == nil {
		blobs = []interface{}{}
	}

	return map[string]interface{}{
		"centroid_mm":     centroid,
		"mean_height_mm":  meanHeight,
		"sector_coverage": sectorCoverage,
		"blobs":           blobs,
	}
}

// ──────────────────────────────────────────────────────────────────────────
// Circle fit (Kasa method)
// ──────────────────────────────────────────────────────────────────────────

// fitCircleKasa fits a circle to the XY projection of pts (Z is ignored) and
// returns the center, radius, and rms residual in the same units as the input.
//
// The method rests on a change of variables. A circle
//
//	(x-centerX)^2 + (y-centerY)^2 = r^2
//
// expands and regroups into
//
//	(x^2+y^2) + D*x + E*y + F = 0,  D = -2cx, E = -2cy, F = centerX^2+centerY^2-r^2
//
// which is *linear* in D, E, F even though the original is nonlinear in
// centerX, centerY, r. So the fit is one closed-form 3x3 least-squares solve: no
// initial guess, no iteration, no failure to converge.
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
