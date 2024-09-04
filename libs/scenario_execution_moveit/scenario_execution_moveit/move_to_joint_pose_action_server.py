import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.client import GoalStatus
from joint_to_pose.action import JointToPose
from rclpy.action.server import ServerGoalHandle
from pymoveit2 import MoveIt2State
from scenario_execution_moveit.moveit_common import MoveIt2Interface
from rclpy.callback_groups import ReentrantCallbackGroup
from scenario_execution_moveit import wx200
from rclpy.executors import MultiThreadedExecutor

class MoveToJointPoseNode(Node):
    def __init__(self):
        super().__init__("move_to_joint_pose")

        self.moveit2 = MoveIt2Interface(
            node=self,
            joint_names=wx200.joint_names(),
            base_link_name=wx200.base_link_name(),
            end_effector_name=wx200.end_effector_name(),
            group_name=wx200.MOVE_GROUP_ARM,
            callback_group=ReentrantCallbackGroup()
        )
        self.moveit2.planner_id = "RRTConnect"
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5

        self.move_to_joint_pose_server_ = ActionServer(
            self, 
            JointToPose, 
            "move_to_joint_pose",
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("Action server move to joint pose has been started.")

    def goal_callback(self, goal_request: JointToPose.Goal):
        self.get_logger().info("Received a goal request")
        current_state = self.moveit2.query_state()
        if current_state == MoveIt2State.EXECUTING:
            self.get_logger().info("Another motion is in progress. Rejecting the goal...")
            return GoalResponse.REJECT
        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        target = goal_handle.request.joint_positions
        self.get_logger().info(f"Executing the goal...{target}")
        result = JointToPose.Result()
        self.moveit2.move_to_configuration(target)
        timeout = 1
        start_time = time.time()
        future = None
        
        while future is None:
            if time.time() - start_time > timeout:
                self.get_logger().info("Timeout reached while waiting for the future.")
                goal_handle.abort()
                result.error_code = 6
                return result
            if self.moveit2.query_state() == MoveIt2State.EXECUTING:
                future = self.moveit2.get_execution_future()
            time.sleep(0.01)
        while not future.done():
            time.sleep(0.01)

        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded.")
            goal_handle.succeed()
            result.error_code = 1
        elif future.result().status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Goal aborted.")
            goal_handle.abort()
            result.error_code = 6
        elif future.result().status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal canceled.")
            goal_handle.canceled()
            result.error_code = 3 

        return result

def main(args=None):
    rclpy.init(args=args)
    node = MoveToJointPoseNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()

