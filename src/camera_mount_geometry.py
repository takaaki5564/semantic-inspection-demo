"""Flange-local visual bracket, in meters; no physics or optical pose changes.

The verified UR10e flange faces +X (wrist mesh ends near X=0). A small
base plate, upright and crossbar form a connected mount to the camera body.
"""

MOUNT_CUBES = (
    ("CameraMountPlate", (0.002, 0, 0), (0.012, 0.06, 0.06), (0.25, 0.28, 0.3)),
    ("CameraMountUpright", (0.01, 0, 0.06), (0.025, 0.03, 0.105), (0.25, 0.28, 0.3)),
    ("CameraBracket", (0.09, 0, 0.1), (0.18, 0.03, 0.025), (0.25, 0.28, 0.3)),
    ("CameraHousing", (0.18, 0, 0.12), (0.06, 0.05, 0.06), (0.05, 0.45, 0.65)),
    ("CameraLens", (0.18, 0, 0.09), (0.032, 0.032, 0.015), (0.06, 0.07, 0.08)),
)
