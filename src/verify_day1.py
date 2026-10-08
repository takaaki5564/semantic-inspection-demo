"""Verify saved Day 1 evidence without starting Isaac Sim."""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image

from capture_metadata import validate_capture_pair


def verify(output_dir):
    output_dir = Path(output_dir).resolve()
    metadata = json.loads((output_dir / "camera_poses.json").read_text(encoding="utf-8"))
    if metadata["scene"]["inspection_evaluated"] is not False:
        raise ValueError("Day 1 must not claim inspection completion")
    records = metadata["captures"]
    images = []
    for record in records:
        path = (output_dir / record["image"]).resolve()
        if not path.is_relative_to(output_dir):
            raise ValueError("Image must be inside the output directory")
        with Image.open(path) as image:
            if image.mode != "RGB":
                raise ValueError("Expected saved RGB PNG")
            images.append(np.asarray(image).copy())
    checks = validate_capture_pair(records, images)
    for image in images:
        if list(image.shape[:2]) != metadata["camera"]["resolution_hw"]:
            raise ValueError("Image resolution differs from metadata")
    for key, value in checks.items():
        if not np.isclose(value, metadata["checks"][key]):
            raise ValueError(f"Stored image comparison is inconsistent: {key}")
    return checks


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path, nargs="?", default=Path(__file__).resolve().parents[1] / "outputs")
    args = parser.parse_args()
    result = verify(args.output_dir)
    print(f"PASS: two distinct RGB images, different camera poses, advancing render reference time, consistent transforms | {result}")
