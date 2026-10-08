"""Small explicit requirement -> cause -> method -> capability relations, without Isaac."""

from dataclasses import dataclass
import json
from pathlib import Path


def unique_ids(values, label):
    if (not isinstance(values, list) or not values
            or any(not isinstance(value, str) or not value for value in values)
            or len(values) != len(set(values))):
        raise ValueError(f"{label} must contain nonempty unique string IDs")
    return tuple(values)


@dataclass(frozen=True)
class InspectionSpec:
    spec_id: str
    version: str
    required_region_ids: tuple

    def required_points(self, regions):
        if any(region not in regions for region in self.required_region_ids):
            raise ValueError("Inspection specification refers to unavailable surface regions")
        return {region: regions[region].point_ids for region in self.required_region_ids}


class InspectionKnowledge:
    def __init__(self, document):
        # Copy the document so later edits to a caller's dictionary cannot change a plan.
        self.document = json.loads(json.dumps(document))
        if document["schema_version"] != 1 or document["approval_status"] != "prototype_not_approved":
            raise ValueError("Expected version 1 prototype knowledge, not an approved recipe")
        if not document["source"] or not document["knowledge_version"]:
            raise ValueError("Knowledge requires source and version")
        states = unique_ids(document["observation_states"], "observation_states")
        if set(states) != {"not_captured", "captured", "missing_observation", "inspection_observation_satisfied"}:
            raise ValueError("Knowledge observation states do not match observation evaluator")
        self.specifications = {}
        for spec_id, value in document["specifications"].items():
            if not spec_id or not isinstance(value["version"], str) or not value["version"]:
                raise ValueError("Inspection specification requires ID and version")
            self.specifications[spec_id] = InspectionSpec(
                spec_id, value["version"], unique_ids(value["required_region_ids"], "required_region_ids"))
        if not self.specifications:
            raise ValueError("Knowledge must define inspection specifications")
        versions = [spec.version for spec in self.specifications.values()]
        if len(versions) != len(set(versions)):
            raise ValueError("Specification versions must distinguish different specifications")
        self.method_capabilities = {
            method: unique_ids(value["required_capability_ids"], "required_capability_ids")
            for method, value in document["methods"].items()}
        for required in self.method_capabilities.values():
            if any(capability not in document["capabilities"] for capability in required):
                raise ValueError("Method refers to an undefined capability")
        self.cause_methods = {
            cause: unique_ids(value["eligible_method_ids"], "eligible_method_ids")
            for cause, value in document["missing_observation_causes"].items()}
        for methods in self.cause_methods.values():
            if any(method not in self.method_capabilities for method in methods):
                raise ValueError("Cause refers to an undefined method")

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))

    def specification(self, spec_id):
        if spec_id not in self.specifications:
            raise ValueError(f"Unknown inspection specification: {spec_id}")
        return self.specifications[spec_id]

    def lookup_methods(self, causes, available_capabilities):
        """Return traceable eligibility, including capability failures; no part labels."""
        methods = sorted({method for cause in causes for method in self.cause_methods.get(cause, ())})
        available = set(available_capabilities)
        return [{"method_id": method,
                 "applicable_causes": sorted(cause for cause in causes if method in self.cause_methods.get(cause, ())),
                 "required_capability_ids": list(self.method_capabilities[method]),
                 "missing_capability_ids": sorted(set(self.method_capabilities[method]) - available)}
                for method in methods]

    def provenance(self):
        return {key: self.document[key] for key in ("knowledge_version", "source", "approval_status")}
