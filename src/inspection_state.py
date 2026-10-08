"""Observation state is independent from image capture and from defect decisions."""


class ObservationState:
    def __init__(self, part_geometry_version, inspection_spec_version, required_point_ids):
        self.reset(part_geometry_version, inspection_spec_version, required_point_ids)

    def reset(self, part_geometry_version, inspection_spec_version, required_point_ids):
        required = {region: frozenset(ids) for region, ids in required_point_ids.items()}
        if not required or any(not points for points in required.values()):
            raise ValueError("Inspection specification must require nonempty surface samples")
        self.part_geometry_version = part_geometry_version
        self.inspection_spec_version = inspection_spec_version
        self.required = required
        self.observed = {region: set() for region in required}
        self.capture_ids = []
        self.evaluated_capture_ids = []
        self._last_evidence = {region: {} for region in required}

    def record_capture(self, capture_id):
        if capture_id in self.capture_ids:
            raise ValueError("Capture ID already recorded")
        self.capture_ids.append(capture_id)

    def apply_visibility(self, capture_id, report):
        if capture_id not in self.capture_ids:
            raise ValueError("Visibility evidence requires a recorded capture")
        if capture_id in self.evaluated_capture_ids:
            raise ValueError("Capture was already evaluated")
        if (report["part_geometry_version"] != self.part_geometry_version
                or report["inspection_spec_version"] != self.inspection_spec_version):
            raise ValueError("Part/specification changed; reset observation state first")
        if set(report["required_region_ids"]) != set(self.required) or set(report["regions"]) != set(self.required):
            raise ValueError("Visibility evidence does not match required regions")
        if report["evidence_source"] != "derived_from_simulator_geometry":
            raise ValueError("Unexpected visibility evidence source")
        updates = {}
        for region, required in self.required.items():
            points = report["regions"][region]["points"]
            ids = [point["point_id"] for point in points]
            if len(ids) != len(set(ids)) or set(ids) != required:
                raise ValueError("Visibility sample IDs do not match active specification")
            if any(point["visible"] != (point["reason"] == "visible") for point in points):
                raise ValueError("Visibility flag and reason disagree")
            updates[region] = {point["point_id"] for point in points if point["visible"]}
        # Validate the complete report before mutating any observation state.
        for region, visible in updates.items():
            self.observed[region].update(visible)
            self._last_evidence[region] = {
                point["point_id"]: {"reason": point["reason"], "capture_id": capture_id}
                for point in report["regions"][region]["points"]}
        self.evaluated_capture_ids.append(capture_id)

    def missing_evidence(self):
        """Latest evaluated cause for each still-missing point; never a defect label."""
        return {region: {point_id: dict(self._last_evidence[region].get(
                    point_id, {"reason": "not_evaluated", "capture_id": None}))
                         for point_id in sorted(required - self.observed[region])}
                for region, required in self.required.items()}

    def summary(self):
        satisfied = all(self.observed[region] >= required for region, required in self.required.items())
        status = "inspection_observation_satisfied" if satisfied else "missing_observation"
        if not self.evaluated_capture_ids:
            status = "captured" if self.capture_ids else "not_captured"
        return {"part_geometry_version": self.part_geometry_version,
                "inspection_spec_version": self.inspection_spec_version,
                "status": status, "inspection_observation_satisfied": satisfied,
                "defect_decision": "not_evaluated", "capture_ids": list(self.capture_ids),
                "regions": {region: {"observed_count": len(self.observed[region]), "total_count": len(required),
                                     "coverage": len(self.observed[region]) / len(required),
                                     "missing_point_ids": sorted(required - self.observed[region])}
                            for region, required in self.required.items()}}
