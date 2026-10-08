"""Evaluator-owned region presentation data; no simulator or GUI dependencies."""

from dataclasses import asdict, dataclass
import json
from pathlib import Path


def load_region_metadata(path=None):
    path = Path(path) if path else Path(__file__).resolve().parents[1]/"config/inspection_regions.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    if document["schema_version"] != 1 or not document["source"]:
        raise ValueError("Unsupported region metadata")
    return document["regions"]


@dataclass(frozen=True)
class InspectionRegionState:
    region_id: str
    name: str
    prim_path: str
    state: str
    required: bool
    observed_count: int | None
    total_count: int | None
    coverage: float | None
    next_capture_target: bool


def region_snapshot(observation, surface_regions, metadata, plan=None):
    """Derive states only from validated cumulative evidence, never predicted coverage."""
    summary = observation.summary()
    action = plan.get("action") if plan and plan["status"] == "ready" else None
    targets = action["predicted_new_point_ids"] if action else {}
    if not set(observation.required) <= set(surface_regions):
        raise ValueError("Required inspection region is absent from the scene")
    result = []
    for region_id, surface in sorted(surface_regions.items()):
        info = metadata[region_id]
        if info["prim_path"] != surface.prim_path or not info["name"].strip():
            raise ValueError("Region metadata does not match the actual surface")
        required = region_id in observation.required
        counts = summary["regions"].get(region_id)
        if not required:
            state = "not_required"
        elif counts["observed_count"] == counts["total_count"]:
            state = "confirmed"
        elif counts["observed_count"]:
            state = "partial"
        elif observation.evaluated_capture_ids:
            state = "needs_recheck"
        else:
            state = "unconfirmed"
        result.append(asdict(InspectionRegionState(
            region_id, info["name"], surface.prim_path, state, required,
            counts["observed_count"] if required else None,
            counts["total_count"] if required else None,
            counts["coverage"] if required else None,
            required and bool(targets.get(region_id)))))
    return result
