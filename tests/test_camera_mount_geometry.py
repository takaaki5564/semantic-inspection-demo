from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from arm_camera_evidence import DEFAULT_MOUNT
from camera_mount_geometry import MOUNT_CUBES


class CameraMountGeometryTests(unittest.TestCase):
    def test_bracket_is_a_connected_solid_chain_from_flange_to_housing_and_lens(self):
        bounds = {name:(np.array(position)-np.array(size)/2, np.array(position)+np.array(size)/2)
                  for name,position,size,_ in MOUNT_CUBES}
        visited = {"CameraMountPlate"}
        while True:
            added = {b for a in visited for b in bounds
                     if np.all(np.minimum(bounds[a][1],bounds[b][1])-np.maximum(bounds[a][0],bounds[b][0]) > 0)}
            if added <= visited:
                break
            visited |= added
        self.assertEqual(visited, set(bounds))
        low,high = bounds["CameraMountPlate"]
        self.assertTrue(np.all(low < 0) and np.all(high > 0))

    def test_mount_solids_leave_camera_optical_origin_and_forward_ray_clear(self):
        mount = np.asarray(DEFAULT_MOUNT)
        origin, forward = mount[:3,3], -mount[:3,2]
        # The camera looks along flange -Z; all camera solids at this X/Y
        # remain behind its optical center, including the new lens front.
        for _,position,size,_ in MOUNT_CUBES:
            low,high = np.array(position)-np.array(size)/2, np.array(position)+np.array(size)/2
            if np.all(origin[:2] >= low[:2]) and np.all(origin[:2] <= high[:2]):
                self.assertGreater(low[2], origin[2])
        np.testing.assert_allclose(forward, (0,0,-1), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
