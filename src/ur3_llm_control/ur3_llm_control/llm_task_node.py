"""ROS 2 node for natural-language task planning and skill execution - Week 3."""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .llm_planner import LLMPlanner, PlannerError
from .robot_skills import RobotSkills
from .task_validator import PlanValidator


class LLMTaskNode(Node):
    def __init__(self):
        super().__init__("llm_task_planner")
        for name, default in (
            ("command", ""), ("api_key", ""),
            ("endpoint", "https://9router.com/v1/chat/completions"),
            ("model", "gpt-4o-mini"), ("execute", True),
            ("group_name", "ur_manipulator"),
            ("student_id", "23020770"),
        ):
            self.declare_parameter(name, default)
            
        self.done = threading.Event()
        self.planner = LLMPlanner(
            endpoint=self.get_parameter("endpoint").value,
            model=self.get_parameter("model").value,
            api_key=self.get_parameter("api_key").value,
            student_id=self.get_parameter("student_id").value,
        )
        self.validator = PlanValidator()
        
        # Nhận trạng thái môi trường từ Camera
        self.world_state = None
        self.sub_world_state = self.create_subscription(
            String, "/world_state", self.world_state_callback, 10
        )

    def world_state_callback(self, msg):
        try:
            self.world_state = json.loads(msg.data)
        except Exception:
            pass

    @staticmethod
    def step_name(step):
        args = [value for key, value in step.items() if key != "skill"]
        return f"{step['skill']}({', '.join(args)})"

    def wait_for_world_state(self, timeout=10.0):
        self.get_logger().info("Waiting for perception data from Camera...")
        start_time = time.time()
        while self.world_state is None and (time.time() - start_time) < timeout:
            time.sleep(0.2)
        if self.world_state is None:
            self.get_logger().warn("Timeout waiting for camera! Proceeding with empty state.")
            return {}
        self.get_logger().info("Camera perception data received successfully.")
        return self.world_state

    def run_task(self, command):
        self.get_logger().info(f"USER COMMAND:\n{command}")
        
        # 1. Thu nhận trạng thái môi trường hiện tại từ Camera
        current_world_state = self.wait_for_world_state()

        # 2. Gọi LLM sinh kế hoạch dựa trên lệnh và tình trạng zone
        try:
            payload = self.planner.create_plan(command, current_world_state)
        except PlannerError as exc:
            self.get_logger().error(f"PLANNER_FAILED: {exc}")
            return False

        # 3. Kiểm định tính hợp lệ của kế hoạch qua Task Validator
        checked = self.validator.validate(payload)
        if not checked.valid:
            self.get_logger().error(f"PLAN_REJECTED: {checked.error}")
            return False

        self.get_logger().info("LLM PLAN:\n" + "\n".join(
            f"{i}. {self.step_name(step)}" for i, step in enumerate(checked.plan, 1)))

        if not self.get_parameter("execute").value:
            self.get_logger().info("EXECUTION SKIPPED (execute=false)")
            return True

        # 4. Thực thi kế hoạch qua RobotSkills
        try:
            skills = RobotSkills(self, self.get_parameter("group_name").value)
            # Cập nhật thông tin đối tượng & vùng thực tế từ Camera cho RobotSkills
            if current_world_state and hasattr(skills, "update_world_state"):
                skills.update_world_state(current_world_state)
        except RuntimeError as exc:
            self.get_logger().error(f"SKILL_INIT_FAILED: {exc}")
            return False

        for index, step in enumerate(checked.plan, 1):
            result = skills.execute(step)
            suffix = f" — {result.detail}" if result.detail else ""
            self.get_logger().info(f"{self.step_name(step):<28} {result.status}{suffix}")
            if result.status != "SUCCESS":
                self.get_logger().error(f"TASK FAILED at step {index}")
                return False

        self.get_logger().info("TASK SUCCESS")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = LLMTaskNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=False)
    thread.start()
    command = node.get_parameter("command").value
    try:
        if not command:
            command = input("USER COMMAND: ")
        node.run_task(command)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        executor.shutdown()
        thread.join(timeout=3.0)
        node.destroy_node()
        rclpy.shutdown()