package objectgeometry

import (
	"context"
	"fmt"
	"image"
	"image/color"
	"image/draw"
	"math"
	"time"

	"github.com/golang/geo/r3"
	"golang.org/x/image/font"
	"golang.org/x/image/font/basicfont"
	"golang.org/x/image/math/fixed"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// The vision test card polls CaptureAllFromCamera about once a second; the
// perception pipeline takes seconds. Analysis is cached and refreshed at most
// this often, with the cached geometry drawn on every fresh frame.
const overlayRefreshInterval = 3 * time.Second

// overlayVessel is one analyzed vessel's drawable state.
type overlayVessel struct {
	centerX, centerY, rimZ, radius float64
	fill                           [numSectors]float64 // fraction of each wedge with contents
	heightMM                       [numSectors]float64 // contents depth per wedge, where present
	meanHeight                     float64
	analyzed                       bool // false until analysis has succeeded
}

// refreshOverlay re-runs detection + analysis if the cache is stale, using the
// service's own internals (no RPC hop).
func (s *objectGeometryShapeFit) refreshOverlay(ctx context.Context) {
	s.overlayMu.Lock()
	stale := time.Since(s.overlayAt) >= overlayRefreshInterval
	if stale {
		s.overlayAt = time.Now() // claim the refresh so concurrent polls skip it
	}
	s.overlayMu.Unlock()
	if !stale {
		return
	}

	results, err := s.detect(ctx, s.camera)
	if err != nil {
		s.logger.Debugf("overlay: detect failed: %v", err)
		return
	}
	var vessels []overlayVessel
	for _, r := range results {
		v := overlayVessel{centerX: r.CenterX, centerY: r.CenterY, rimZ: r.CenterZ, radius: r.Radius}
		baseline := s.pickBaseline(r.CenterX, r.CenterY)
		analysis := analyzeRegion(r.Cloud, r.CenterX, r.CenterY, r.CenterZ, r.Radius, minContentsHeightMM, baseline)
		if fill, ok := analysis["sector_fill"].([]interface{}); ok {
			for i := 0; i < len(fill) && i < numSectors; i++ {
				v.fill[i], _ = toFloat(fill[i])
			}
			v.analyzed = true
		}
		if hts, ok := analysis["sector_height_mm"].([]interface{}); ok {
			for i := 0; i < len(hts) && i < numSectors; i++ {
				v.heightMM[i], _ = toFloat(hts[i])
			}
		}
		v.meanHeight, _ = toFloat(analysis["mean_height_mm"])
		vessels = append(vessels, v)
	}

	s.overlayMu.Lock()
	s.overlayVessels = vessels
	s.overlayMu.Unlock()
}

// annotatedFrame grabs a fresh source frame and draws the cached analysis —
// the fitted circle, sector wedges, and per-sector coverage values — on it.
func (s *objectGeometryShapeFit) annotatedFrame(ctx context.Context, extra map[string]interface{}) (image.Image, error) {
	src, err := s.sourceImage(ctx, extra)
	if err != nil {
		return nil, fmt.Errorf("overlay source image: %w", err)
	}

	s.refreshOverlay(ctx)
	s.overlayMu.Lock()
	vessels := make([]overlayVessel, len(s.overlayVessels))
	copy(vessels, s.overlayVessels)
	s.overlayMu.Unlock()
	if len(vessels) == 0 {
		return src, nil // nothing analyzed yet: pass the frame through
	}

	proj, err := s.projector(ctx)
	if err != nil {
		s.logger.Debugf("overlay: no projection available: %v", err)
		return src, nil
	}

	canvas := image.NewRGBA(src.Bounds())
	draw.Draw(canvas, src.Bounds(), src, src.Bounds().Min, draw.Src)
	for _, v := range vessels {
		drawVessel(canvas, v, proj)
	}
	return canvas, nil
}

// sourceImage tolerates cameras that implement Images but not Image, like the
// detect-crop camera the segmenter reads from.
func (s *objectGeometryShapeFit) sourceImage(ctx context.Context, extra map[string]interface{}) (image.Image, error) {
	if s.cam == nil {
		return nil, errNoCamera
	}
	if imgs, _, err := s.cam.Images(ctx, nil, extra); err == nil && len(imgs) > 0 {
		return imgs[0].Image(ctx)
	}
	return camera.DecodeImageFromCamera(ctx, "", extra, s.cam)
}

// projector builds a world-point-to-pixel function from the camera's
// intrinsics and the frame system's world→camera transform.
func (s *objectGeometryShapeFit) projector(ctx context.Context) (func(x, y, z float64) (int, int, bool), error) {
	props, err := s.cam.Properties(ctx)
	if err != nil {
		return nil, err
	}
	intr := props.IntrinsicParams
	if err := intr.CheckValid(); err != nil {
		return nil, fmt.Errorf("overlay needs camera intrinsics: %w", err)
	}
	wtc, err := s.fsService.TransformPose(ctx,
		referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewZeroPose()),
		s.cameraFrame, nil)
	if err != nil {
		return nil, fmt.Errorf("world->camera transform: %w", err)
	}
	worldToCam := wtc.Pose()
	return func(x, y, z float64) (int, int, bool) {
		pc := spatialmath.Compose(worldToCam, spatialmath.NewPoseFromPoint(r3.Vector{X: x, Y: y, Z: z})).Point()
		if pc.Z <= 0 {
			return 0, 0, false // behind the camera
		}
		u, v := intr.PointToPixel(pc.X, pc.Y, pc.Z)
		return int(u), int(v), true
	}, nil
}

// ── drawing ───────────────────────────────────────────────────────────────

var (
	overlayLine  = color.RGBA{R: 255, G: 220, B: 0, A: 255}   // circle + wedge boundaries
	overlayText  = color.RGBA{R: 255, G: 255, B: 255, A: 255} // value labels
	overlayBadge = color.RGBA{A: 170}                         // label background for contrast
	overlayFill  = color.RGBA{R: 0, G: 220, B: 90}            // wedge fill; alpha scales with fill
)

const labelScale = 2 // 2x the 7x13 base font so values are readable

// drawVessel draws the fitted circle, the sector wedge boundaries, and each
// sector's coverage value. All geometry is computed as world points and
// projected, so image orientation is handled by the projection itself.
func drawVessel(img *image.RGBA, v overlayVessel, proj func(x, y, z float64) (int, int, bool)) {
	cxPx, cyPx, centerOK := proj(v.centerX, v.centerY, v.rimZ)

	// Shade each wedge by its fill — the fraction of the wedge that actually
	// has contents. Fill is absolute (bare wedge 0, fully covered 1), so an
	// empty vessel naturally shades nothing; no extra gating needed.
	if v.analyzed && centerOK {
		for k := range numSectors {
			f := v.fill[k]
			if f <= 0 {
				continue
			}
			poly := wedgePolygon(v, k, proj, cxPx, cyPx)
			// Steep ramp for contrast: a nearly-bare wedge is a whisper of
			// green, a full one unmistakably solid.
			alpha := uint8(15 + f*185)
			fillPolygon(img, poly, overlayFill, alpha)
		}
	}

	// Circle outline: project points around the rim and connect them.
	const steps = 96
	var prevX, prevY int
	var havePrev bool
	for i := 0; i <= steps; i++ {
		a := 2 * math.Pi * float64(i) / steps
		x, y, ok := proj(v.centerX+v.radius*math.Cos(a), v.centerY+v.radius*math.Sin(a), v.rimZ)
		if !ok {
			havePrev = false
			continue
		}
		if havePrev {
			drawLine(img, prevX, prevY, x, y, overlayLine)
		}
		prevX, prevY, havePrev = x, y, true
	}

	// Wedge boundaries: world angles k*(2π/numSectors) from +X, matching the
	// sector indexing in analyzeRegion.
	if centerOK {
		for k := range numSectors {
			a := 2 * math.Pi * float64(k) / numSectors
			x, y, ok := proj(v.centerX+v.radius*math.Cos(a), v.centerY+v.radius*math.Sin(a), v.rimZ)
			if ok {
				drawLine(img, cxPx, cyPx, x, y, overlayLine)
			}
		}
	}

	// Per-sector labels at mid-wedge, ~65% of the radius out: the fill
	// fraction, plus the contents depth where there are contents.
	if v.analyzed {
		for k := range numSectors {
			mid := 2 * math.Pi * (float64(k) + 0.5) / numSectors
			x, y, ok := proj(v.centerX+0.65*v.radius*math.Cos(mid), v.centerY+0.65*v.radius*math.Sin(mid), v.rimZ)
			if !ok {
				continue
			}
			label := fmt.Sprintf("%.0f%%", v.fill[k]*100)
			if v.fill[k] > 0 {
				label = fmt.Sprintf("%.0f%% %.0fmm", v.fill[k]*100, v.heightMM[k])
			}
			drawLabel(img, x, y, label)
		}
	}

	// Summary at the projected center: mean contents height. Clamp tiny
	// negatives so an empty vessel reads "h=0mm", not "h=-0mm".
	if centerOK && v.analyzed {
		mh := v.meanHeight
		if math.Abs(mh) < 0.5 {
			mh = 0
		}
		drawLabel(img, cxPx, cyPx, fmt.Sprintf("h=%.0fmm", mh))
	}
}

// drawLine draws a 1px Bresenham line clipped to the image bounds.
func drawLine(img *image.RGBA, x0, y0, x1, y1 int, c color.Color) {
	dx, dy := abs(x1-x0), -abs(y1-y0)
	sx, sy := 1, 1
	if x0 >= x1 {
		sx = -1
	}
	if y0 >= y1 {
		sy = -1
	}
	err := dx + dy
	b := img.Bounds()
	for {
		if image.Pt(x0, y0).In(b) {
			img.Set(x0, y0, c)
		}
		if x0 == x1 && y0 == y1 {
			return
		}
		if e2 := 2 * err; e2 >= dy {
			err += dy
			x0 += sx
		} else {
			err += dx
			y0 += sy
		}
	}
}

// wedgePolygon builds sector k's projected outline: the vessel center plus
// the arc from angle k/numSectors to (k+1)/numSectors of a full turn.
func wedgePolygon(v overlayVessel, k int, proj func(x, y, z float64) (int, int, bool), cxPx, cyPx int) []image.Point {
	const arcSteps = 16
	poly := []image.Point{{X: cxPx, Y: cyPx}}
	for i := 0; i <= arcSteps; i++ {
		a := 2 * math.Pi * (float64(k) + float64(i)/arcSteps) / numSectors
		x, y, ok := proj(v.centerX+v.radius*math.Cos(a), v.centerY+v.radius*math.Sin(a), v.rimZ)
		if ok {
			poly = append(poly, image.Point{X: x, Y: y})
		}
	}
	return poly
}

// fillPolygon alpha-blends a solid color over the polygon's interior using
// even-odd scanline filling.
func fillPolygon(img *image.RGBA, poly []image.Point, c color.RGBA, alpha uint8) {
	if len(poly) < 3 {
		return
	}
	minY, maxY := poly[0].Y, poly[0].Y
	for _, p := range poly {
		minY = min(minY, p.Y)
		maxY = max(maxY, p.Y)
	}
	b := img.Bounds()
	minY = max(minY, b.Min.Y)
	maxY = min(maxY, b.Max.Y-1)

	for y := minY; y <= maxY; y++ {
		// x-intersections of the scanline with polygon edges
		var xs []int
		j := len(poly) - 1
		for i := range poly {
			pi, pj := poly[i], poly[j]
			if (pi.Y <= y && pj.Y > y) || (pj.Y <= y && pi.Y > y) {
				t := float64(y-pi.Y) / float64(pj.Y-pi.Y)
				xs = append(xs, pi.X+int(t*float64(pj.X-pi.X)))
			}
			j = i
		}
		sortInts(xs)
		for i := 0; i+1 < len(xs); i += 2 {
			x0, x1 := max(xs[i], b.Min.X), min(xs[i+1], b.Max.X-1)
			for x := x0; x <= x1; x++ {
				blendPixel(img, x, y, c, alpha)
			}
		}
	}
}

// blendPixel alpha-blends color c over the pixel at (x, y).
func blendPixel(img *image.RGBA, x, y int, c color.RGBA, alpha uint8) {
	old := img.RGBAAt(x, y)
	a := uint32(alpha)
	blend := func(o, n uint8) uint8 {
		return uint8((uint32(o)*(255-a) + uint32(n)*a) / 255)
	}
	img.SetRGBA(x, y, color.RGBA{
		R: blend(old.R, c.R),
		G: blend(old.G, c.G),
		B: blend(old.B, c.B),
		A: 255,
	})
}

func sortInts(xs []int) {
	for i := 1; i < len(xs); i++ {
		for j := i; j > 0 && xs[j] < xs[j-1]; j-- {
			xs[j], xs[j-1] = xs[j-1], xs[j]
		}
	}
}

// drawLabel draws text centered at (x, y), scaled up labelScale times, on a
// translucent dark badge so it reads over any background.
func drawLabel(img *image.RGBA, x, y int, s string) {
	face := basicfont.Face7x13
	w := font.MeasureString(face, s).Ceil()
	h := face.Metrics().Height.Ceil()

	// Render the string once at native size into a small scratch image.
	scratch := image.NewRGBA(image.Rect(0, 0, w+2, h+2))
	d := font.Drawer{
		Dst:  scratch,
		Src:  image.NewUniform(overlayText),
		Face: face,
		Dot:  fixed.P(1, 1+face.Metrics().Ascent.Ceil()),
	}
	d.DrawString(s)

	// Badge behind the scaled text.
	sw, sh := (w+2)*labelScale, (h+2)*labelScale
	x0, y0 := x-sw/2, y-sh/2
	for yy := y0 - 2; yy < y0+sh+2; yy++ {
		for xx := x0 - 2; xx < x0+sw+2; xx++ {
			if image.Pt(xx, yy).In(img.Bounds()) {
				blendPixel(img, xx, yy, color.RGBA{}, overlayBadge.A)
			}
		}
	}

	// Blit the scratch text scaled up, nearest-neighbor.
	for sy := range sh {
		for sx := range sw {
			if _, _, _, a := scratch.At(sx/labelScale, sy/labelScale).RGBA(); a == 0 {
				continue
			}
			if image.Pt(x0+sx, y0+sy).In(img.Bounds()) {
				img.SetRGBA(x0+sx, y0+sy, overlayText)
			}
		}
	}
}

func abs(a int) int {
	if a < 0 {
		return -a
	}
	return a
}
