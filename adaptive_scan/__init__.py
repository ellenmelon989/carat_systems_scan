"""
adaptive_scan/ — the edge-following, open-loop-safe coarse scan path (see
docs/adaptive_scan_spec.md). Never trusts commanded position (step count)
as physical position — each raster row detects its own wafer boundaries
from a live sensor signal instead. Kept separate from scan/ (the
precision, absolute-position path) by design; both share the top-level
motion/ and readers/ packages.
"""
