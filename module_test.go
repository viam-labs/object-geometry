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

// ringFloor lays points at height z in the annulus between innerR and outerR
// from the origin — a stand-in for a vessel's sloped wall.
func ringFloor(innerR, outerR, step, z float64) [][3]float64 {
	var out [][3]float64
	for x := -outerR; x <= outerR; x += step {
		for y := -outerR; y <= outerR; y += step {
			if d := math.Hypot(x, y); d >= innerR && d <= outerR {
				out = append(out, [3]float64{x, y, z})
			}
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
	raw, ok := result["sector_coverage"].([]interface{})
	if !ok {
		t.Fatalf("no sector_coverage in result: %v", result)
	}
	var s [8]float64
	for i, v := range raw {
		s[i] = v.(float64)
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
	floorGrid, floorLevel := g.floorSnapshot()
	return &vesselBaseline{
		centerX:    centerX,
		centerY:    centerY,
		radius:     radius,
		floorLevel: floorLevel,
		floorGrid:  floorGrid,
		halfCells:  g.halfCells,
		gridCells:  g.occupied,
	}
}

// A pile of contents on the +X side should pull the centroid toward +X, away
// from the vessel center at the origin.
func TestCentroidPulledTowardPile(t *testing.T) {
	const floorZ, pileZ, rimZ = 100, 120, 130

	pts := floor(-80, 80, -80, 80, 5, floorZ)              // flat floor everywhere
	pts = append(pts, floor(40, 70, -15, 15, 3, pileZ)...) // taller pile at +X

	result := analyzeRegion(buildCloud(t, pts), 0, 0, rimZ, 100, 0, nil)
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

	result := analyzeRegion(buildCloud(t, pts), 0, 0, rimZ, 100, 0, nil)
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
	result := analyzeRegion(buildCloud(t, pts), 0, 0, 130, 100, 0, nil)
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

	result := analyzeRegion(buildCloud(t, pts), 0, 0, rimZ, 100, 0, nil)
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

	result := analyzeRegion(buildCloud(t, pts), 0, 0, rimZ, 100, 0, nil)
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

// A noisy empty vessel must yield no blobs once the min-contents-height gate is on,
// while a real pile above the gate still registers.
func TestMinContentsHeightGate(t *testing.T) {
	const rimZ = 140
	// Empty floor at 100 with bounded ±3mm noise (6mm peak-to-peak, no RNG).
	var pts [][3]float64
	for x := -80.0; x <= 80; x += 3 {
		for y := -80.0; y <= 80; y += 3 {
			jitter := math.Mod(math.Abs(x*7+y*13), 6) - 3 // [-3, 3)
			pts = append(pts, [3]float64{x, y, 100 + jitter})
		}
	}

	// Empty noisy vessel: an 8mm gate (> the 6mm noise) yields no blobs.
	empty := analyzeRegion(buildCloud(t, pts), 0, 0, rimZ, 100, 8, nil)
	if n := len(blobsOf(t, empty)); n != 0 {
		t.Errorf("empty noisy vessel with 8mm gate: got %d blobs, want 0", n)
	}

	// A real 20mm pile clears the gate and is still found.
	pts = append(pts, floor(30, 60, -15, 15, 3, 120)...)
	withPile := analyzeRegion(buildCloud(t, pts), 0, 0, rimZ, 100, 8, nil)
	if n := len(blobsOf(t, withPile)); n == 0 {
		t.Error("20mm pile with 8mm gate: got 0 blobs, want at least 1")
	}
}

// With contents covering the whole floor, the lowest visible cell is no longer the
// true floor, so the estimated-floor path understates height. A baseline
// captured while empty restores the true reference, giving greater mean height.
func TestBaselineFloorRaisesMeanHeight(t *testing.T) {
	const emptyFloorZ, layerZ, pileZ, rimZ = 100, 112, 130, 150

	// Baseline: the empty vessel, floor at 100 across the whole interior.
	emptyPts := floor(-90, 90, -90, 90, 3, emptyFloorZ)
	baseline := makeBaseline(t, emptyPts, 0, 0, rimZ, 100)

	// Cooking: a layer of contents covers the entire floor at 112, plus a pile.
	foodPts := floor(-90, 90, -90, 90, 3, layerZ)
	foodPts = append(foodPts, floor(30, 55, -12, 12, 3, pileZ)...)
	cloud := buildCloud(t, foodPts)

	estimated := analyzeRegion(cloud, 0, 0, rimZ, 100, 0, nil)["mean_height_mm"].(float64)
	withBase := analyzeRegion(cloud, 0, 0, rimZ, 100, 0, baseline)["mean_height_mm"].(float64)

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

// A degenerate fit with a sub-inset radius must not panic on a negative grid
// width; buildHeightGrid should return an empty grid and analyzeRegion an
// empty result.
func TestTinyRadiusDoesNotPanic(t *testing.T) {
	pts := floor(-5, 5, -5, 5, 1, 100)
	cloud := buildCloud(t, pts)

	for _, radius := range []float64{0.5, insetMM, insetMM - 0.1, math.NaN(), math.Inf(1)} {
		g := buildHeightGrid(cloud, 0, 0, 130, radius)
		if g.occupied != 0 {
			t.Errorf("radius %.2f: got %d occupied cells, want 0", radius, g.occupied)
		}
		result := analyzeRegion(cloud, 0, 0, 130, radius, 0, nil)
		if hgt := result["mean_height_mm"].(float64); hgt != 0 {
			t.Errorf("radius %.2f: mean_height_mm = %.1f, want 0 (empty result)", radius, hgt)
		}
	}
}

// The derived band should track the object's top wherever it sits, capturing
// the rim and excluding a floor that's more than zBandWidthMM below it.
func TestRimBandDerivedFromTop(t *testing.T) {
	s := &objectGeometryShapeFit{} // fixedBand false → derived band

	// Pan at an arbitrary height: rim ring at 300, floor 50mm lower at 250.
	pts := ringFloor(120, 130, 3, 300)
	pts = append(pts, ringFloor(0, 120, 3, 250)...)
	zMin, zMax := s.rimBand(buildCloud(t, pts))

	if !(zMin <= 300 && 300 <= zMax) {
		t.Errorf("rim (300) not in derived band [%.0f, %.0f]", zMin, zMax)
	}
	if zMin <= 250 {
		t.Errorf("floor (250) not excluded: band [%.0f, %.0f] reaches it", zMin, zMax)
	}
}

// A pinned band is returned verbatim regardless of the cloud.
func TestRimBandPinned(t *testing.T) {
	s := &objectGeometryShapeFit{fixedBand: true, zMinMM: 95, zMaxMM: 135}
	pts := ringFloor(0, 100, 3, 999) // top far from the pinned band
	zMin, zMax := s.rimBand(buildCloud(t, pts))
	if zMin != 95 || zMax != 135 {
		t.Errorf("pinned band = [%.0f, %.0f], want [95, 135]", zMin, zMax)
	}
}

func TestDedupeOverlapping(t *testing.T) {
	// Two overlapping fits of one vessel (the phantom's center is inside the real
	// vessel) collapse to the stronger one; a genuinely separate vessel is kept.
	results := []shapeResult{
		{CenterX: -65, CenterY: 385, Radius: 132, PointCnt: 19277}, // real vessel
		{CenterX: -76, CenterY: 328, Radius: 81, PointCnt: 3000},   // phantom, overlaps
		{CenterX: 400, CenterY: 385, Radius: 120, PointCnt: 8000},  // a separate vessel
	}

	kept := dedupeOverlapping(results)
	if len(kept) != 2 {
		t.Fatalf("kept %d fits, want 2 (one vessel + one phantom collapsed, plus the separate vessel)", len(kept))
	}
	// The phantom (radius 81) must be the one dropped.
	for _, k := range kept {
		if k.Radius == 81 {
			t.Errorf("phantom fit (radius 81) survived: %+v", kept)
		}
	}
}

// The floor level is the flat bottom, read from the vessel's center, not the
// mean of the whole interior. A bowl with a flat-bottom disc at 100 and a
// sloped wall ring rising to 120 should report ~100 even though the walls
// (which a real sensor samples more densely) would pull a mean or percentile up.
func TestFloorSnapshotReportsFlatBottom(t *testing.T) {
	pts := ringFloor(0, 55, 3, 100)                 // flat-bottom disc at 100
	pts = append(pts, ringFloor(55, 90, 3, 120)...) // sloped wall ring at 120
	g := buildHeightGrid(buildCloud(t, pts), 0, 0, 140, 100)
	_, floorLevel := g.floorSnapshot()
	if math.Abs(floorLevel-100) > 3 {
		t.Errorf("floor level = %.1f, want ~100 (the flat bottom, not the bottom+wall mean)", floorLevel)
	}
}

// ── circle fit ────────────────────────────────────────────────────────────

// ringPoints places n points evenly on a circular arc at height z.
func ringPoints(cx, cy, r, z float64, n int, startDeg, endDeg float64) [][3]float64 {
	out := make([][3]float64, 0, n)
	start := startDeg * math.Pi / 180
	end := endDeg * math.Pi / 180
	for i := range n {
		a := start + (end-start)*float64(i)/float64(n)
		out = append(out, [3]float64{cx + r*math.Cos(a), cy + r*math.Sin(a), z})
	}
	return out
}

// The fit must recover a known full ring exactly (up to float error).
func TestFitCircleKasaExact(t *testing.T) {
	pts := ringPoints(-65, 385, 128, 105, 720, 0, 360)
	var vecs []r3.Vector
	for _, p := range pts {
		vecs = append(vecs, r3.Vector{X: p[0], Y: p[1], Z: p[2]})
	}
	cx, cy, r, rms := fitCircleKasa(vecs)
	if math.Abs(cx+65) > 0.1 || math.Abs(cy-385) > 0.1 {
		t.Errorf("center = (%.2f, %.2f), want (-65, 385)", cx, cy)
	}
	if math.Abs(r-128) > 0.1 {
		t.Errorf("radius = %.2f, want 128", r)
	}
	if rms > 0.1 {
		t.Errorf("rms = %.3f, want ~0 for perfect ring", rms)
	}
}

// A partial arc (the camera never sees the far rim) must still recover the
// center and radius closely. Kasa reads slightly small on short arcs; a 240°
// arc should stay within a couple mm.
func TestFitCircleKasaPartialArc(t *testing.T) {
	pts := ringPoints(0, 0, 100, 50, 480, 0, 240)
	var vecs []r3.Vector
	for _, p := range pts {
		vecs = append(vecs, r3.Vector{X: p[0], Y: p[1], Z: p[2]})
	}
	cx, cy, r, _ := fitCircleKasa(vecs)
	if math.Hypot(cx, cy) > 2 {
		t.Errorf("center = (%.2f, %.2f), want within 2mm of origin", cx, cy)
	}
	if math.Abs(r-100) > 2 {
		t.Errorf("radius = %.2f, want 100 ±2", r)
	}
}

// Deterministic noise on the ring should degrade rms but not bias the fit.
func TestFitCircleKasaNoisy(t *testing.T) {
	var vecs []r3.Vector
	for i := range 720 {
		a := 2 * math.Pi * float64(i) / 720
		jitter := math.Mod(float64(i)*7.3, 6) - 3 // radial noise in [-3, 3)
		r := 100 + jitter
		vecs = append(vecs, r3.Vector{X: r * math.Cos(a), Y: r * math.Sin(a), Z: 50})
	}
	cx, cy, r, rms := fitCircleKasa(vecs)
	if math.Hypot(cx, cy) > 1 {
		t.Errorf("center = (%.2f, %.2f), want near origin", cx, cy)
	}
	if math.Abs(r-100) > 1 {
		t.Errorf("radius = %.2f, want ~100", r)
	}
	if rms < 0.5 || rms > 3.5 {
		t.Errorf("rms = %.2f, want ~sqrt(mean sq of ±3mm noise)", rms)
	}
}

// ── blob shape ────────────────────────────────────────────────────────────

// An elongated pile should report a major axis along its long direction and
// major/minor lengths near its real footprint.
func TestBlobPCAOrientationAndExtent(t *testing.T) {
	const floorZ, pileZ, rimZ = 100, 130, 145
	pts := floor(-80, 80, -80, 80, 4, floorZ)
	// Long thin pile along X: 80mm x 20mm centered at origin.
	pts = append(pts, floor(-40, 40, -10, 10, 3, pileZ)...)

	blobs := blobsOf(t, analyzeRegion(buildCloud(t, pts), 0, 0, rimZ, 100, 0, nil))
	if len(blobs) != 1 {
		t.Fatalf("found %d blobs, want 1", len(blobs))
	}
	b := blobs[0]
	axis := b["major_axis_deg"].(float64)
	// Along X means axis ~0 or ~180 (mod 180).
	if m := math.Min(math.Abs(axis), math.Abs(180-math.Abs(axis))); m > 10 {
		t.Errorf("major axis = %.0f°, want along X (0° or 180°)", axis)
	}
	maj := b["major_len_mm"].(float64)
	minl := b["minor_len_mm"].(float64)
	if math.Abs(maj-80) > 12 {
		t.Errorf("major length = %.0f, want ~80", maj)
	}
	if math.Abs(minl-20) > 10 {
		t.Errorf("minor length = %.0f, want ~20", minl)
	}
	if maj <= minl {
		t.Errorf("major (%.0f) should exceed minor (%.0f)", maj, minl)
	}
}

// Two well-separated piles must come back as two distinct blobs.
func TestBlobDetectionTwoPiles(t *testing.T) {
	const floorZ, pileZ, rimZ = 100, 130, 145
	pts := floor(-80, 80, -80, 80, 4, floorZ)
	pts = append(pts, floor(30, 60, -15, 15, 3, pileZ)...)   // pile at +X
	pts = append(pts, floor(-60, -30, -15, 15, 3, pileZ)...) // pile at -X

	blobs := blobsOf(t, analyzeRegion(buildCloud(t, pts), 0, 0, rimZ, 100, 0, nil))
	if len(blobs) != 2 {
		t.Fatalf("found %d blobs, want 2 separated piles", len(blobs))
	}
}

// ── config validation ─────────────────────────────────────────────────────

func TestValidateZBandRules(t *testing.T) {
	f := func(v float64) *float64 { return &v }
	base := Config{CameraFrame: "cam", Segmenter: "seg"}

	ok := base
	if _, _, err := ok.Validate("p"); err != nil {
		t.Errorf("no z band set: unexpected error %v", err)
	}

	half := base
	half.ZMinMM = f(95)
	if _, _, err := half.Validate("p"); err == nil {
		t.Error("z_min without z_max: want error, got nil")
	}

	inverted := base
	inverted.ZMinMM, inverted.ZMaxMM = f(135), f(95)
	if _, _, err := inverted.Validate("p"); err == nil {
		t.Error("z_min >= z_max: want error, got nil")
	}

	good := base
	good.ZMinMM, good.ZMaxMM = f(95), f(135)
	if _, _, err := good.Validate("p"); err != nil {
		t.Errorf("valid z band: unexpected error %v", err)
	}
}

// ── rim band robustness ───────────────────────────────────────────────────

// A few stray points above the rim (a handle tip, a reflection) must not drag
// the derived band up past the rim: the 95th percentile ignores them.
func TestRimBandIgnoresHighOutliers(t *testing.T) {
	s := &objectGeometryShapeFit{}
	pts := ringFloor(120, 130, 3, 300)                // rim ring at 300
	pts = append(pts, floor(0, 10, 0, 10, 3, 400)...) // small outlier patch far above

	zMin, zMax := s.rimBand(buildCloud(t, pts))
	if !(zMin <= 300 && 300 <= zMax) {
		t.Errorf("rim (300) not in band [%.0f, %.0f]; outliers dragged it up", zMin, zMax)
	}
}

// ── baseline grid mismatch ────────────────────────────────────────────────

// A baseline captured at a different radius has a different grid size; the
// per-cell floor cannot align, so it must fall back to the scalar floor level
// rather than misindex.
func TestBaselineGridMismatchFallsBack(t *testing.T) {
	baseline := makeBaseline(t, floor(-60, 60, -60, 60, 3, 100), 0, 0, 140, 60)
	other := buildHeightGrid(buildCloud(t, floor(-90, 90, -90, 90, 3, 100)), 0, 0, 140, 100)
	if baseline.halfCells == other.halfCells {
		t.Fatal("test setup: grids should differ in size")
	}

	floorAt := baseline.floorAt(other.halfCells)
	// Every lookup must return the scalar floor, including out-of-range indices
	// for the baseline's own smaller grid.
	for _, rc := range [][2]int{{0, 0}, {other.halfCells, other.halfCells}, {2 * other.halfCells, 2 * other.halfCells}} {
		if got := floorAt(rc[0], rc[1]); got != baseline.floorLevel {
			t.Errorf("floorAt(%d,%d) = %.1f, want scalar fallback %.1f", rc[0], rc[1], got, baseline.floorLevel)
		}
	}
}
