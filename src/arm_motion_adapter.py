"""Isaac Sim 6.1 / cuMotion adapter for the isolated Step 3 motion probe."""

import time

import numpy as np

from arm_kinematics import pose_error
from arm_motion import IK_TOLERANCES, motion_limits, within_limits
from capture_metadata import camera_look_at, rigid_transform


class ArmMotionAdapter:
    """Manual fixed physics steps; target commands only, no joint teleportation."""

    dt = 1 / 240

    def __init__(self, app, asset, model_dir, reference, *, headless):
        import omni.usd
        from pxr import Gf, UsdGeom, UsdPhysics
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.robot_motion.cumotion import load_cumotion_robot
        from arm_preflight_adapter import load_scene, ROBOT_PATH

        self.app, self.headless, self.sm = app, headless, SimulationManager
        self.robot = load_cumotion_robot(model_dir)
        self.names = list(self.robot.controlled_joint_names)
        self.limits = motion_limits(model_dir, self.names)
        if self.robot.kinematics.base_frame_name() != "base_link" or "flange" not in self.robot.robot_description.tool_frame_names():
            raise ValueError("Unexpected model base/tool frame")
        self.static_scene = load_scene(app, asset, headless=headless)
        stage = omni.usd.get_context().get_stage()
        # Scene placement before physics; never used to reach a motion goal.
        stage.GetPrimAtPath(ROBOT_PATH).GetAttribute("xformOp:translate").Set(Gf.Vec3d(0, 0, 0.4))
        floor = stage.GetPrimAtPath("/World/Floor")
        UsdPhysics.CollisionAPI.Apply(floor)
        pedestal = UsdGeom.Cube.Define(stage, "/World/Pedestal")
        pedestal.CreateSizeAttr(1.0)
        pedestal.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.195))
        pedestal.AddScaleOp().Set(Gf.Vec3f(0.24, 0.24, 0.4))
        pedestal.CreateDisplayColorAttr([Gf.Vec3f(0.22, 0.25, 0.28)])
        # Pedestal is a visual marker; the authored fixed root joint anchors the robot.
        camera = UsdGeom.Camera(stage.GetPrimAtPath("/World/OverviewCamera"))
        camera.MakeMatrixXform().Set(Gf.Matrix4d(camera_look_at((2.5, -2.5, 2), (0.5, 0.1, 0.65)).T.tolist()))
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr(9.81)
        self.sm.set_physics_sim_device("cpu")
        self.sm.set_physics_dt(self.dt)
        self.arm = Articulation(ROBOT_PATH)
        self.arm.set_solver_iteration_counts(position_counts=32, velocity_counts=4)
        self.stiffness = [200000.0]*3 + [50000.0]*3
        self.damping = [5000.0]*3 + [1000.0]*3
        self.arm.set_dof_gains(stiffnesses=self.stiffness, dampings=self.damping)
        self.bodies = RigidPrim([reference["base_prim_path"], reference["joints"][-1]["child_path"]])
        self.T_wrist_flange = rigid_transform(reference["T_leaf_flange"])
        # Supported manual initialization/stepping keeps app.update from adding physics steps.
        self.sm.initialize_physics()
        if not self.arm.is_physics_tensor_entity_valid() or not self.bodies.is_physics_tensor_entity_valid():
            raise RuntimeError("Physics tensor readback unavailable")
        if set(self.arm.dof_names) != set(self.names) or len(self.arm.dof_names) != len(self.names):
            raise ValueError("Articulation/model joint names differ")
        self.read_indices = [self.arm.dof_names.index(n) for n in self.names]
        self.write_indices = [self.names.index(n) for n in self.arm.dof_names]
        self.step_count = 0
        self.render_interval = 32 if headless else 8
        self.last_render_wall = time.monotonic()
        self.settings = {"physics_dt_seconds": self.dt, "physics_device": "cpu", "gravity_m_s2": 9.81,
                         "solver_position_iterations": 32, "solver_velocity_iterations": 4,
                         "drive_stiffness": self.stiffness, "drive_damping": self.damping,
                         "model_joint_order": self.names, "articulation_joint_order": list(self.arm.dof_names),
                         "physics_clock": "SimulationManager manual fixed steps; timeline remains stopped",
                         "pose_source": "PhysX tensor base/wrist world poses composed with fixed USD T_wrist_flange",
                         "T_wrist_flange": self.T_wrist_flange.tolist(),
                         "base_scene_translation_m": [0, 0, 0.4],
                         "fabric_enabled": self.sm.is_fabric_enabled(),
                         "pedestal_collision_enabled": False, "floor_collision_enabled": True,
                         "collision_avoidance": False,
                         "self_collision_enabled": self.arm.get_enabled_self_collisions().numpy().tolist()}

    def read_state(self):
        from pxr import Gf

        if not self.arm.is_physics_tensor_entity_valid() or not self.bodies.is_physics_tensor_entity_valid():
            raise RuntimeError("Physics tensor readback became invalid")
        positions, orientations = (a.numpy().copy() for a in self.bodies.get_world_poses())
        transforms = []
        for p, q in zip(positions, orientations):
            q = q.astype(float) / np.linalg.norm(q)
            matrix = Gf.Matrix4d(1)
            matrix.SetRotate(Gf.Quatd(float(q[0]), Gf.Vec3d(*q[1:].tolist())))
            transform = np.asarray(matrix).T.copy()
            transform[:3, 3] = p
            transforms.append(rigid_transform(transform))
        return {"simulation_time_s": float(self.sm.get_simulation_time()),
                "physics_step_index": int(self.sm.get_num_physics_steps()),
                "joints_rad": self.arm.get_dof_positions().numpy()[0, self.read_indices].astype(float).tolist(),
                "velocities_rad_s": self.arm.get_dof_velocities().numpy()[0, self.read_indices].astype(float).tolist(),
                "T_world_base": transforms[0].tolist(),
                "T_world_flange": (transforms[1] @ self.T_wrist_flange).tolist()}

    def model_pose(self, q):
        return self.robot.kinematics.pose(q, "flange").matrix()

    def solve_ik(self, target, start):
        import cumotion

        seeds = [start.copy()]
        for offset in ((0, -0.4, 0.4, 0, 0.3, 0), (0, 0.4, -0.4, 0, -0.3, 0)):
            seeds.append(np.clip(start + offset, self.limits["lower"], self.limits["upper"]))
        attempts, candidates = [], []
        for seed in seeds:
            config = cumotion.IkConfig()
            config.cspace_seeds = [seed.tolist()]
            config.max_num_descents = 1
            config.sampling_seed = 0
            config.position_tolerance = IK_TOLERANCES["position_m"]
            config.orientation_tolerance = IK_TOLERANCES["orientation_rad"]
            config.bfgs_gradient_norm_termination = 1e-10
            config.bfgs_max_iterations = 500
            config.bfgs_cspace_limit_biasing = cumotion.IkConfig.CSpaceLimitBiasing.DISABLE
            answer = cumotion.solve_ik(self.robot.kinematics, cumotion.Pose3(target), "flange", config)
            attempt = {"seed_rad": seed.tolist(), "success": bool(answer.success), "num_descents": int(answer.num_descents)}
            if answer.success:
                q = np.array(answer.cspace_position, dtype=float, copy=True)
                # Equivalent revolute angles nearest the current physical state, within hard limits.
                for i in range(len(q)):
                    choices = [q[i] + 2*np.pi*k for k in range(-2, 3)
                               if self.limits["lower"][i] <= q[i]+2*np.pi*k <= self.limits["upper"][i]]
                    if choices:
                        q[i] = min(choices, key=lambda angle: abs(angle-start[i]))
                error = pose_error(self.model_pose(q), target)
                attempt.update(joints_rad=q.tolist(), **error)
                if within_limits(q, self.limits) and error["position_error_m"] <= IK_TOLERANCES["position_m"] and error["orientation_error_rad"] <= IK_TOLERANCES["orientation_rad"]:
                    candidates.append(q)
            attempts.append(attempt)
        if not candidates:
            return {"success": False, "attempts": attempts}
        q = min(candidates, key=lambda item: float(np.linalg.norm(item-start)))
        return {"success": True, "joints_rad": q.tolist(), "attempts": attempts,
                "collision_aware": False, "selection": "nearest successful bounded solution among three deterministic seeds"}

    def command(self, joints):
        self.arm.set_dof_position_targets(np.asarray(joints)[self.write_indices])

    def step(self):
        self.sm.step(steps=1, update_fabric=self.sm.is_fabric_enabled())
        self.step_count += 1
        if self.step_count % self.render_interval == 0:
            if not self.headless:
                remaining = self.render_interval*self.dt - (time.monotonic()-self.last_render_wall)
                if remaining > 0:
                    time.sleep(remaining)
            self.app.update()
            self.last_render_wall = time.monotonic()

    def is_running(self):
        return self.app.is_running()
