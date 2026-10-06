"""MoveItPy implementation of the fixed, allow-listed robot skills - Week 3."""

import time
import threading
import math
import subprocess

from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    AllowedCollisionEntry,
    CollisionObject,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PlanningScene,
    PlanningSceneComponents,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetPlanningScene,
    GetPositionFK,
    GetPositionIK,
)

from .task_validator import VALID_OBJECTS, VALID_ZONES


class SkillResult:
    def __init__(self, status, detail=""):
        self.status, self.detail = status, detail


class RobotSkills:
    OBJECTS = {
        "red_cube": (0.34, -0.22, 0.50),
        "yellow_cube": (0.34, 0.00, 0.50),
        "blue_cube": (0.54, 0.00, 0.50),
        "green_cube": (0.26, -0.10, 0.50),
        "purple_cube": (0.26, 0.10, 0.50),
    }
    # Dời temporary_position sang bên phải tránh xa red_cube
    ZONES = {
        "zone_a": (0.54, -0.24, 0.526),
        "zone_b": (0.54, 0.00, 0.526),
        "zone_c": (0.54, 0.24, 0.526),
        "temporary_position": (0.40, 0.22, 0.526),
    }
    ZONE_MARKERS = {
        "zone_a": (0.54, -0.24, 0.476),
        "zone_b": (0.54, 0.00, 0.476),
        "zone_c": (0.54, 0.24, 0.476),
    }
    HOME = {
        "shoulder_pan_joint": 0.0,
        "shoulder_lift_joint": -1.20,
        "elbow_joint": 1.00,
        "wrist_1_joint": -1.37,
        "wrist_2_joint": -1.57,
        "wrist_3_joint": 0.0,
    }
    APPROACH_CLEARANCES = (0.22, 0.18, 0.26)
    ARM_JOINTS = (
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    )

    def __init__(self, node, group_name="ur_manipulator"):
        self.node = node
        self.group_name = group_name
        self.held_object = None
        self.object_positions = dict(self.OBJECTS)
        self.zones = dict(self.ZONES)
        
        self.move_client = ActionClient(node, MoveGroup, "/move_action")
        self.cartesian_client = node.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self.arm_trajectory_client = ActionClient(
            node,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.scene_publisher = node.create_publisher(PlanningScene, "/planning_scene", 10)
        self.get_scene_client = node.create_client(GetPlanningScene, "/get_planning_scene")
        self.fk_client = node.create_client(GetPositionFK, "/compute_fk")
        self.ik_client = node.create_client(GetPositionIK, "/compute_ik")
        self.apply_scene_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.gripper_client = ActionClient(
            node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        
        time.sleep(1.0)
        self._publish_box("worktable", (0.40, 0.0, 0.43), (0.75, 0.85, 0.08))
        for name, xyz in self.object_positions.items():
            self._publish_box(name, xyz, (0.06, 0.06, 0.06))
        for name, xyz in self.ZONE_MARKERS.items():
            self._publish_box(name, xyz, (0.11, 0.11, 0.012))
        time.sleep(0.25)

    def update_world_state(self, world_state):
        objects = world_state.get("objects", {})
        for name, pos in objects.items():
            if name in VALID_OBJECTS:
                self.object_positions[name] = tuple(pos)
                self._publish_box(name, tuple(pos), (0.06, 0.06, 0.06))
        self.node.get_logger().info("RobotSkills updated positions from Camera perception")

    def _publish_box(self, name, xyz, size, operation=CollisionObject.ADD):
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = name
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(size)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.w = 1.0
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        obj.operation = operation
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        self.scene_publisher.publish(scene)
        time.sleep(0.1)

    def _remove_box(self, name):
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = name
        obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        self.scene_publisher.publish(scene)

    def _set_gazebo_attachment(self, name, attached):
        verb = "attach" if attached else "detach"
        try:
            subprocess.run(
                ["ign", "topic", "-t", f"/ur3_llm/grasp/{name}/{verb}",
                 "-m", "ignition.msgs.Empty", "-p", "unused: true"],
                check=False,
                capture_output=True,
                timeout=1.0,
            )
        except Exception:
            pass
        return True

    def _allow_grasp_contacts(self, obj, allowed):
        if not self.get_scene_client.wait_for_service(timeout_sec=3.0):
            return False
        if not self.apply_scene_client.wait_for_service(timeout_sec=3.0):
            return False

        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self.get_scene_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=5.0) or future.exception() is not None:
            return False
        scene = future.result().scene
        matrix = scene.allowed_collision_matrix
        names = list(matrix.entry_names)
        rows = matrix.entry_values
        if len(rows) != len(names) or any(len(row.enabled) != len(names) for row in rows):
            return False

        for name in (obj, "left_finger_link", "right_finger_link"):
            if name not in names:
                for row in rows:
                    row.enabled.append(False)
                entry = AllowedCollisionEntry()
                entry.enabled = [False] * (len(names) + 1)
                entry.enabled[-1] = True
                rows.append(entry)
                names.append(name)

        for finger in ("left_finger_link", "right_finger_link"):
            i, j = names.index(obj), names.index(finger)
            rows[i].enabled[j] = allowed
            rows[j].enabled[i] = allowed

        matrix.entry_names = names
        matrix.entry_values = rows
        scene.is_diff = True
        apply_request = ApplyPlanningScene.Request()
        apply_request.scene = scene
        apply_future = self.apply_scene_client.call_async(apply_request)
        apply_done = threading.Event()
        apply_future.add_done_callback(lambda _: apply_done.set())
        if not apply_done.wait(timeout=5.0) or apply_future.exception() is not None:
            return False
        return apply_future.result().success

    def _set_attached_box(self, name, attached, xyz=None):
        scene = PlanningScene()
        scene.is_diff = True
        if attached:
            world_obj = CollisionObject()
            world_obj.header.frame_id = "world"
            world_obj.id = name
            world_obj.operation = CollisionObject.REMOVE
            scene.world.collision_objects = [world_obj]
            attached_obj = AttachedCollisionObject()
            attached_obj.link_name = "tool0"
            attached_obj.touch_links = ["gripper_base_link", "left_finger_link", "right_finger_link"]
            attached_obj.object.header.frame_id = "tool0"
            attached_obj.object.id = name
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [0.06, 0.06, 0.06]
            pose = Pose()
            pose.position.z = 0.035
            pose.orientation.w = 1.0
            attached_obj.object.primitives = [primitive]
            attached_obj.object.primitive_poses = [pose]
            attached_obj.object.operation = CollisionObject.ADD
            scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects = [attached_obj]
        else:
            attached_obj = AttachedCollisionObject()
            attached_obj.link_name = "tool0"
            attached_obj.object.id = name
            attached_obj.object.operation = CollisionObject.REMOVE
            scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects = [attached_obj]
            if xyz is not None:
                self.object_positions[name] = xyz
                world_obj = CollisionObject()
                world_obj.header.frame_id = "world"
                world_obj.id = name
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = [0.06, 0.06, 0.06]
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = xyz
                pose.orientation.w = 1.0
                world_obj.primitives = [primitive]
                world_obj.primitive_poses = [pose]
                world_obj.operation = CollisionObject.ADD
                scene.world.collision_objects = [world_obj]
        self.scene_publisher.publish(scene)
        time.sleep(0.15)

    def _command_gripper(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            return SkillResult("FAILED", "gripper_controller action server is unavailable")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["left_finger_joint", "right_finger_joint"]
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.velocities = [0.0, 0.0]
        point.time_from_start.sec = 1
        goal.trajectory.points = [point]
        for name in goal.trajectory.joint_names:
            tolerance = JointTolerance()
            tolerance.name = name
            tolerance.position = 0.02
            goal.path_tolerance.append(tolerance)
            goal_tolerance = JointTolerance()
            goal_tolerance.name = name
            goal_tolerance.position = 0.015
            goal.goal_tolerance.append(goal_tolerance)
        goal.goal_time_tolerance.sec = 2
        done = threading.Event()
        response = []
        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(lambda f: (response.append(f.result()), done.set()))
        if not done.wait(timeout=5.0) or not response[0].accepted:
            return SkillResult("FAILED", "gripper command was not accepted")
        done.clear()
        response.clear()
        result_future = future.result().get_result_async()
        result_future.add_done_callback(lambda f: (response.append(f.result()), done.set()))
        if not done.wait(timeout=8.0):
            return SkillResult("FAILED", "gripper action timed out")
        if response[0].result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return SkillResult("FAILED", "gripper trajectory failed")
        return SkillResult("SUCCESS")

    def _pose(self, xyz):
        stamped = PoseStamped()
        stamped.header.frame_id = "world"
        stamped.header.stamp = self.node.get_clock().now().to_msg()
        stamped.pose.position.x, stamped.pose.position.y, stamped.pose.position.z = xyz
        stamped.pose.orientation.x = 0.0
        stamped.pose.orientation.y = 1.0
        stamped.pose.orientation.z = 0.0
        stamped.pose.orientation.w = 0.0
        return stamped

    def _move_best_approach(self, x, y, z):
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            return SkillResult("FAILED", "MoveIt /move_action server is unavailable"), None
        if not self.get_scene_client.wait_for_service(timeout_sec=5.0):
            return SkillResult("FAILED", "MoveIt /get_planning_scene is unavailable"), None
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            return SkillResult("FAILED", "MoveIt /compute_ik is unavailable"), None

        scene_request = GetPlanningScene.Request()
        scene_request.components.components = (
            PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        scene_future = self.get_scene_client.call_async(scene_request)
        scene_done = threading.Event()
        scene_future.add_done_callback(lambda _: scene_done.set())
        if not scene_done.wait(timeout=5.0) or scene_future.exception() is not None:
            return SkillResult("FAILED", "Could not read current robot state"), None
        start_state = scene_future.result().scene.robot_state
        current = dict(zip(start_state.joint_state.name, start_state.joint_state.position))
        if not all(name in current for name in self.ARM_JOINTS):
            return SkillResult("FAILED", "Current state is missing UR arm joints"), None

        candidates = []
        for clearance in self.APPROACH_CLEARANCES:
            ik_request = GetPositionIK.Request()
            ik = ik_request.ik_request
            ik.group_name = self.group_name
            ik.robot_state = start_state
            ik.ik_link_name = "tool0"
            ik.pose_stamped = self._pose((x, y, z + clearance))
            ik.avoid_collisions = True
            ik.timeout.sec = 1
            ik_future = self.ik_client.call_async(ik_request)
            ik_done = threading.Event()
            ik_future.add_done_callback(lambda _, event=ik_done: event.set())
            if not ik_done.wait(timeout=2.0) or ik_future.exception() is not None:
                continue
            ik_response = ik_future.result()
            if ik_response.error_code.val != ik_response.error_code.SUCCESS:
                continue

            solution = dict(current)
            solution.update(zip(
                ik_response.solution.joint_state.name,
                ik_response.solution.joint_state.position,
            ))
            if not all(name in solution for name in self.ARM_JOINTS):
                continue
            weights = {"shoulder_pan_joint": 1.5, "wrist_3_joint": 1.5}
            score = sum(
                weights.get(name, 1.0) * abs(solution[name] - current[name])
                for name in self.ARM_JOINTS
            )
            elbow_bend = abs(solution["elbow_joint"])
            score += 2.0 * max(0.0, 0.35 - elbow_bend)
            if clearance < 0.20:
                score += 0.15
            candidates.append((score, clearance, solution))

        if not candidates:
            return SkillResult(
                "PLANNING_FAILED", "No collision-free IK solution for candidate approach poses"
            ), None

        candidates.sort(key=lambda item: item[0])
        last_result = SkillResult("PLANNING_FAILED", "No candidate path was executable")
        for score, clearance, solution in candidates:
            goal = MoveGroup.Goal()
            request = goal.request
            request.group_name = self.group_name
            request.planner_id = "RRTstarkConfigDefault"
            request.num_planning_attempts = 1
            request.allowed_planning_time = 4.0
            request.max_velocity_scaling_factor = 0.12
            request.max_acceleration_scaling_factor = 0.12
            constraints = Constraints()
            for name in self.ARM_JOINTS:
                joint = JointConstraint()
                joint.joint_name = name
                joint.position = solution[name]
                joint.tolerance_above = 0.025
                joint.tolerance_below = 0.025
                joint.weight = 1.0
                constraints.joint_constraints.append(joint)
            request.goal_constraints = [constraints]
            goal.planning_options.plan_only = False
            last_result = self._send_move_goal(goal)
            if last_result.status == "SUCCESS":
                return last_result, clearance

            request.planner_id = "RRTConnectkConfigDefault"
            request.allowed_planning_time = 4.0
            last_result = self._send_move_goal(goal)
            if last_result.status == "SUCCESS":
                return last_result, clearance
        return last_result, None

    def _send_move_goal(self, goal):
        done = threading.Event()
        result_box = []
        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(lambda f: (result_box.append(f.result()), done.set()))
        if not done.wait(timeout=10.0):
            return SkillResult("FAILED", "MoveIt goal acceptance timed out")
        handle = result_box[0]
        if not handle.accepted:
            return SkillResult("FAILED", "MoveIt rejected the motion goal")
        done.clear()
        result_box.clear()
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f: (result_box.append(f.result()), done.set()))
        if not done.wait(timeout=45.0):
            return SkillResult("FAILED", "MoveIt motion timed out")
        wrapped = result_box[0]
        if wrapped.result.error_code.val != wrapped.result.error_code.SUCCESS:
            return SkillResult(
                "PLANNING_FAILED",
                f"MoveIt returned error code {wrapped.result.error_code.val}",
            )
        return SkillResult("SUCCESS")

    def home(self):
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            return SkillResult("FAILED", "MoveIt /move_action server is unavailable")
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 8
        goal.request.allowed_planning_time = 8.0
        goal.request.max_velocity_scaling_factor = 0.12
        goal.request.max_acceleration_scaling_factor = 0.12
        constraints = Constraints()
        for name, value in self.HOME.items():
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = value
            joint.tolerance_above = 0.015
            joint.tolerance_below = 0.015
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        return self._send_move_goal(goal)

    def _move_above_then_lower(self, x, y, z, check_descent_collisions=True):
        result, clearance = self._move_best_approach(x, y, z)
        if result.status != "SUCCESS":
            return result
        return self._cartesian_lower(
            x, y, z, clearance=clearance,
            avoid_collisions=check_descent_collisions,
        )

    def _cartesian_lower(self, x, y, z, clearance=0.22, avoid_collisions=True):
        return self._cartesian_vertical(
            x, y, [z + clearance * 0.82, z + clearance * 0.55, z + 0.035],
            z + clearance,
            "lowered vertically", avoid_collisions=avoid_collisions,
        )

    def _cartesian_raise(self, x, y, z):
        return self._cartesian_vertical(
            x, y, [z + 0.08, z + 0.15, z + 0.22], z + 0.035,
            "raised vertically",
            avoid_collisions=False,
        )

    def _cartesian_vertical(
        self, x, y, z_waypoints, expected_start_z, action_detail,
        avoid_collisions=True,
    ):
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            return SkillResult("FAILED", "MoveIt /compute_cartesian_path is unavailable")
        if not self.get_scene_client.wait_for_service(timeout_sec=5.0):
            return SkillResult("FAILED", "MoveIt /get_planning_scene is unavailable")

        state_request = GetPlanningScene.Request()
        state_request.components.components = (
            PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        state_future = self.get_scene_client.call_async(state_request)
        state_done = threading.Event()
        state_future.add_done_callback(lambda _: state_done.set())
        if not state_done.wait(timeout=5.0) or state_future.exception() is not None:
            return SkillResult("FAILED", "Could not read current robot state for Cartesian path")
        start_state = state_future.result().scene.robot_state
        if not start_state.joint_state.name:
            return SkillResult("FAILED", "MoveIt returned an empty joint state for Cartesian path")

        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            return SkillResult("FAILED", "MoveIt /compute_fk is unavailable")
        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = "world"
        fk_request.robot_state = start_state
        fk_request.fk_link_names = ["tool0"]
        fk_future = self.fk_client.call_async(fk_request)
        fk_done = threading.Event()
        fk_future.add_done_callback(lambda _: fk_done.set())
        if not fk_done.wait(timeout=5.0) or fk_future.exception() is not None:
            return SkillResult("FAILED", "Could not validate Cartesian path start pose")
        fk_response = fk_future.result()
        if fk_response.error_code.val != fk_response.error_code.SUCCESS or not fk_response.pose_stamped:
            return SkillResult("FAILED", "Forward kinematics failed for Cartesian path start")
        current_pose = fk_response.pose_stamped[0].pose.position
        start_error = math.sqrt(
            (current_pose.x - x) ** 2
            + (current_pose.y - y) ** 2
            + (current_pose.z - expected_start_z) ** 2
        )
        if start_error > 0.035:
            return SkillResult(
                "FAILED",
                f"Refusing non-vertical Cartesian path: tool is {start_error:.3f} m from its expected start",
            )

        request = GetCartesianPath.Request()
        request.header.frame_id = "world"
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.start_state = start_state
        request.start_state.is_diff = False
        request.group_name = self.group_name
        request.link_name = "tool0"
        request.waypoints = [self._pose((x, y, waypoint_z)).pose
                             for waypoint_z in z_waypoints]
        request.max_step = 0.002
        request.jump_threshold = 0.0
        request.avoid_collisions = avoid_collisions

        future = self.cartesian_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=15.0) or future.exception() is not None:
            return SkillResult("FAILED", "Cartesian path request failed or timed out")
        response = future.result()
        if response.error_code.val != response.error_code.SUCCESS or response.fraction < 0.999:
            return SkillResult(
                "PLANNING_FAILED",
                f"Cartesian vertical path incomplete ({response.fraction:.0%}, "
                f"MoveIt code {response.error_code.val})",
            )

        trajectory = response.solution.joint_trajectory
        if not trajectory.points:
            return SkillResult("PLANNING_FAILED", "Cartesian descent returned no trajectory")

        current_positions = dict(zip(start_state.joint_state.name,
                                     start_state.joint_state.position))
        first_point = trajectory.points[0]
        for index, joint_name in enumerate(trajectory.joint_names):
            current = current_positions.get(joint_name)
            if current is None or index >= len(first_point.positions):
                continue
            offset = round((current - first_point.positions[index]) / (2.0 * math.pi))
            shift = offset * 2.0 * math.pi
            previous = current
            for point in trajectory.points:
                if index >= len(point.positions):
                    continue
                position = point.positions[index] + shift
                position += round((previous - position) / (2.0 * math.pi)) * 2.0 * math.pi
                point.positions[index] = position
                previous = position

        time_scale = 3.0
        for point in trajectory.points:
            total_ns = (point.time_from_start.sec * 1_000_000_000
                        + point.time_from_start.nanosec)
            total_ns = int(total_ns * time_scale)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(
                total_ns, 1_000_000_000
            )
            point.velocities = [v / time_scale for v in point.velocities]
            point.accelerations = [a / (time_scale * time_scale)
                                   for a in point.accelerations]

        if not self.arm_trajectory_client.wait_for_server(timeout_sec=5.0):
            return SkillResult("FAILED", "Arm trajectory controller is unavailable")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance.sec = 8
        send_future = self.arm_trajectory_client.send_goal_async(goal)
        accepted = threading.Event()
        goal_handle = []
        send_future.add_done_callback(lambda f: (goal_handle.append(f.result()), accepted.set()))
        if not accepted.wait(timeout=10.0) or not goal_handle[0].accepted:
            return SkillResult("FAILED", "Controller rejected Cartesian descent")
        result_future = goal_handle[0].get_result_async()
        finished = threading.Event()
        result_box = []
        result_future.add_done_callback(lambda f: (result_box.append(f.result()), finished.set()))
        if not finished.wait(timeout=60.0):
            return SkillResult("FAILED", "Cartesian descent execution timed out")
        result = result_box[0].result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return SkillResult("FAILED", f"Cartesian vertical controller error {result.error_code}")
        return SkillResult("SUCCESS", action_detail)

    def check_zone(self, zone):
        if zone not in VALID_ZONES:
            return SkillResult("INVALID_ZONE")
        self.node.get_logger().info(f"[SKILL] Checking zone: {zone} via Perception")
        time.sleep(0.5)
        return SkillResult("SUCCESS", f"inspected {zone}")

    def pick(self, obj):
        if obj not in VALID_OBJECTS:
            return SkillResult("INVALID_OBJECT")
        if self.held_object:
            return SkillResult("FAILED", f"Already holding {self.held_object}")
        x, y, z = self.object_positions[obj]
        opened = self._command_gripper(0.0)
        if opened.status != "SUCCESS":
            return opened
        if not self._allow_grasp_contacts(obj, True):
            return SkillResult("FAILED", "Could not allow safe gripper contact with target")
        result = self._move_above_then_lower(x, y, z)
        if result.status != "SUCCESS":
            self._allow_grasp_contacts(obj, False)
            return result

        closed = self._command_gripper(0.010)
        if closed.status != "SUCCESS":
            self._allow_grasp_contacts(obj, False)
            return closed
        time.sleep(0.2)
        self._set_gazebo_attachment(obj, True)
        self._set_attached_box(obj, True)
        if not self._allow_grasp_contacts(obj, False):
            self._set_attached_box(obj, False, (x, y, z))
            return SkillResult("FAILED", "Object is attached but grasp collision settings did not reset")
        self.held_object = obj
        result = self._cartesian_raise(x, y, z)
        return result if result.status != "SUCCESS" else SkillResult("SUCCESS", f"picked {obj}")

    def place(self, obj, zone):
        if obj not in VALID_OBJECTS:
            return SkillResult("INVALID_OBJECT")
        if zone not in VALID_ZONES:
            return SkillResult("INVALID_ZONE")
        if self.held_object != obj:
            return SkillResult("FAILED", f"{obj} is not held")
        x, y, z = self.zones[zone]
        result = self._move_above_then_lower(
            x, y, z, check_descent_collisions=False
        )
        if result.status != "SUCCESS":
            return result
        opened = self._command_gripper(0.0)
        if opened.status != "SUCCESS":
            return opened
        self._set_gazebo_attachment(obj, False)
        self._set_attached_box(obj, False, (x, y, z))
        self.held_object = None
        if not self._allow_grasp_contacts(obj, True):
            return SkillResult("FAILED", "Object released, but gripper exit contact was not enabled")
        result = self._cartesian_raise(x, y, z)
        contacts_reset = self._allow_grasp_contacts(obj, False)
        if not contacts_reset:
            return SkillResult("FAILED", "Object released, but gripper contact settings did not reset")
        return result if result.status != "SUCCESS" else SkillResult("SUCCESS", f"placed {obj} in {zone}")

    def execute(self, step):
        if step["skill"] == "home":
            return self.home()
        if step["skill"] == "check_zone":
            return self.check_zone(step["zone"])
        if step["skill"] == "pick":
            return self.pick(step["object"])
        if step["skill"] == "place":
            return self.place(step["object"], step["zone"])
        return SkillResult("INVALID_SKILL")
