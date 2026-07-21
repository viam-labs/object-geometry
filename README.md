# Module object-geometry

Fits geometric primitives to segmented point clouds and analyzes what's inside
the vessels it finds.

Given a segmenter that isolates a vessel — a pan, a bowl, a bin — this module
transforms its points into the world frame, finds the rim, and fits a circle
to it: center, radius, and rim height, in world millimeters. It can then
analyze the vessel's contents against a captured empty-vessel baseline: how
much of each sector is covered, how deep the contents are, and where the
distinct piles sit. A built-in visualization draws the analysis onto the
camera frame as a live overlay.

## Models

This module provides the following model(s):

- [`viam:object-geometry:shape-fit`](viam_object-geometry_shape-fit.md) —
  vessel detection, contents analysis, and the analysis overlay
