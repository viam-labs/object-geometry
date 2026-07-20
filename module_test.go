package objectgeometry

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
)

// buildCloud makes a point cloud from (x, y, z) triples in mm.
func buildCloud(t *testing.T, pts [][3]float64) pointcloud.PointCloud {
	t.Helper()
	pc := pointcloud.NewBasicEmpty()
	for _, p := range pts {
		if err := pc.Set(r3.Vector{X: p[0], Y: p[1], Z: p[2]}, pointcloud.NewBasicData()); err != nil {
			t.Fatalf("set point: %v", err)
		}
	}
	return pc
}

// floor lays down a flat grid of points at height z across a square region.
func floor(minX, maxX, minY, maxY, step, z float64) [][3]float64 {
	var out [][3]float64
	for x := minX; x <= maxX; x += step {
		for y := minY; y <= maxY; y += step {
			out = append(out, [3]float64{x, y, z})
		}
	}
	return out
}

func centroidOf(t *testing.T, result map[string]interface{}) (x, y float64) {
	t.Helper()
	c, ok := result["centroid_mm"].(map[string]interface{})
	if !ok {
		t.Fatalf("no centroid_mm in result: %v", result)
	}
	return c["x"].(float64), c["y"].(float64)
}

func sectorsOf(t *testing.T, result map[string]interface{}) [8]float64 {
	t.Helper()
	s, ok := result["sector_coverage"].([8]float64)
	if !ok {
		t.Fatalf("no sector_coverage in result: %v", result)
	}
	return s
}

func blobsOf(t *testing.T, result map[string]interface{}) []map[string]interface{} {
	t.Helper()
	raw, ok := result["blobs"].([]interface{})
	if !ok {
		t.Fatalf("no blobs in result: %v", result)
	}
	out := make([]map[string]interface{}, len(raw))
	for i, b := range raw {
		out[i] = b.(map[string]interface{})
	}
	return out
}

// makeBaseline captures an empty vessel the same way doCaptureBaseline does,
// so tests exercise the real grid and floor snapshot.
func makeBaseline(t *testing.T, pts [][3]float64, centerX, centerY, rimZ, radius float64) *vesselBaseline {
	t.Helper()
	g := buildHeightGrid(buildCloud(t, pts), centerX, centerY, rimZ, radius)
	floorGrid, floorMean := g.floorSnapshot()
	return &vesselBaseline{
		centerX:   centerX,
		centerY:   centerY,
		radius:    radius,
		floorMean: floorMean,
		floorGrid: floorGrid,
		halfCells: g.halfCells,
		gridCells: g.occupied,
	}
}

// A pile of food on the +X side should pull the centroid toward +X, away
// from the vessel center at the origin.
func TestCentroidPulledTowardPile(t *testing.T) {
	const floorZ, pileZ, rimZ = 100, 120, 130

	pts := floor(-80, 80, -80, 80, 5, floorZ)              // flat floor everywhere
	pts = append(pts, floor(40, 70, -15, 15, 3, pileZ)...) // taller pile at +X

	result := analyzeRegion(buildCloud(t, pts), 0, 0, rimZ, 100, nil)
	x, y := centroidOf(t, result)

	if x <= 0 {
		t.Errorf("centroid x = %.1f, want > 0 (pulled toward the +X pile)", x)
	}
	// Floor cells weigh ~nothing (height above floor is 0), so the centroid
	// should sit near the pile's own center around x=55, y=0.
	if math.Abs(y) > 10 {
		t.Errorf("centroid y = %.1f, want near 0 (pile is centered on the x axis)", y)
	}
	t.Logf("centroid = (%.1f, %.1f)", x, y)
}

// Evenly spread contents should leave the centroid near the vessel center.
func TestCentroidEvenIsCentered(t *testing.T) {
	const domeBase, rimZ = 100, 140

	// A shallow symmetric dome centered at the origin: height depends only on
	// distance from center, so there's no directional bias.
	var pts [][3]float64
	for x := -80.0; x <= 80; x += 4 {
		for y := -80.0; y <= 80; y += 4 {
			r := math.Hypot(x, y)
			z := domeBase + math.Max(0, 30-r*0.3)
			pts = append(pts, [3]float64{x, y, z})
		}
	}

	result := analyzeRegion(buildCloud(t, pts), 0, 0, rimZ, 100, nil)
	x, y := centroidOf(t, result)

	if math.Hypot(x, y) > 8 {
		t.Errorf("centroid = (%.1f, %.1f), want near origin for symmetric contents", x, y)
	}
	t.Logf("centroid = (%.1f, %.1f)", x, y)
}

// An empty (perfectly flat) vessel has no center of mass; the guard should
// return the origin rather than dividing by zero.
func TestCentroidEmptyIsOrigin(t *testing.T) {
	pts := floor(-80, 80, -80, 80, 5, 100) // flat floor, nothing on top
	result := analyzeRegion(buildCloud(t, pts), 0, 0, 130, 100, nil)
	x, y := centroidOf(t, result)
	if x != 0 || y != 0 {
		t.Errorf("centroid = (%.1f, %.1f), want (0, 0) for empty vessel", x, y)
	}
}

// A pile placed squarely in sector 0 (the +X wedge, angles 0–45°) should make
// that sector read as the tallest, 1.0.
func TestSectorCoverageDirection(t *testing.T) {
	const floorZ, pileZ, rimZ = 100, 125, 140

	pts := floor(-80, 80, -80, 80, 4, floorZ)
	// Pile centered near angle 22° — the middle of sector 0.
	pts = append(pts, floor(38, 58, 10, 30, 3, pileZ)...)

	result := analyzeRegion(buildCloud(t, pts), 0, 0, rimZ, 100, nil)
	cov := sectorsOf(t, result)

	argmax := 0
	for i, v := range cov {
		if v > cov[argmax] {
			argmax = i
		}
	}
	if argmax != 0 {
		t.Errorf("tallest sector = %d (%v), want sector 0 (the +X wedge)", argmax, cov)
	}
	if cov[0] != 1.0 {
		t.Errorf("sector 0 coverage = %.1f, want 1.0 (the tallest normalizes to 1)", cov[0])
	}
}

// One compact pile should be found as exactly one blob, centered on the pile.
func TestBlobDetectionSinglePile(t *testing.T) {
	const floorZ, pileZ, rimZ = 100, 130, 145

	pts := floor(-80, 80, -80, 80, 4, floorZ)
	pts = append(pts, floor(30, 60, -15, 15, 3, pileZ)...) // pile centered at (45, 0)

	result := analyzeRegion(buildCloud(t, pts), 0, 0, rimZ, 100, nil)
	blobs := blobsOf(t, result)

	if len(blobs) != 1 {
		t.Fatalf("found %d blobs, want 1", len(blobs))
	}
	c := blobs[0]["centroid_mm"].(map[string]interface{})
	bx, by := c["x"].(float64), c["y"].(float64)
	if math.Abs(bx-45) > 10 || math.Abs(by) > 10 {
		t.Errorf("blob centroid = (%.0f, %.0f), want near (45, 0)", bx, by)
	}
}

// With food covering the whole floor, the lowest visible cell is no longer the
// true floor, so the estimated-floor path understates height. A baseline
// captured while empty restores the true reference, giving greater mean height.
func TestBaselineFloorRaisesMeanHeight(t *testing.T) {
	const emptyFloorZ, layerZ, pileZ, rimZ = 100, 112, 130, 150

	// Baseline: the empty vessel, floor at 100 across the whole interior.
	emptyPts := floor(-90, 90, -90, 90, 3, emptyFloorZ)
	baseline := makeBaseline(t, emptyPts, 0, 0, rimZ, 100)

	// Cooking: a layer of food covers the entire floor at 112, plus a pile.
	foodPts := floor(-90, 90, -90, 90, 3, layerZ)
	foodPts = append(foodPts, floor(30, 55, -12, 12, 3, pileZ)...)
	cloud := buildCloud(t, foodPts)

	estimated := analyzeRegion(cloud, 0, 0, rimZ, 100, nil)["mean_height_mm"].(float64)
	withBase := analyzeRegion(cloud, 0, 0, rimZ, 100, baseline)["mean_height_mm"].(float64)

	if withBase <= estimated {
		t.Errorf("mean height: baseline %.0f, estimated %.0f — baseline should be higher "+
			"(it sees the covered floor)", withBase, estimated)
	}
	// The layer sits 12mm above the true floor everywhere, invisible to the
	// estimated path but recovered by the baseline.
	if diff := withBase - estimated; diff < 8 {
		t.Errorf("baseline raised mean height by %.0fmm, want ~12 (the buried layer)", diff)
	}
}

func TestPickBaselineMatchByPosition(t *testing.T) {
	s := &objectGeometryShapeFit{
		baselines: []vesselBaseline{
			{centerX: 0, centerY: 0, radius: 100},
		},
	}

	// Near the center: match.
	if b := s.pickBaseline(5, 5); b == nil {
		t.Error("near center: got nil, want the captured baseline")
	}
	// Beyond the vessel radius: no match.
	if b := s.pickBaseline(500, 500); b != nil {
		t.Errorf("far away: got %v, want nil", b)
	}
}

func TestFloorSnapshotMean(t *testing.T) {
	// Two heights split evenly should average halfway between them.
	pts := floor(-90, 0, -90, 90, 3, 100)               // left half at 100
	pts = append(pts, floor(3, 90, -90, 90, 3, 120)...) // right half at 120
	g := buildHeightGrid(buildCloud(t, pts), 0, 0, 140, 100)
	_, mean := g.floorSnapshot()
	if math.Abs(mean-110) > 3 {
		t.Errorf("floor mean = %.1f, want ~110 (halfway between the two halves)", mean)
	}
}
