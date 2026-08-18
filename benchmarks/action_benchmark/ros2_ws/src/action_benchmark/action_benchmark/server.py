import rclpy
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class BenchServer(Node):
    def __init__(self):
        super().__init__("bench_server")
        self.declare_parameter("n", 5)
        self.group = ReentrantCallbackGroup()
        self.servers = [
            ActionServer(
                self,
                Fibonacci,
                f"/bench_action_{index}",
                self.execute,
                callback_group=self.group,
            )
            for index in range(self.get_parameter("n").value)
        ]

    def execute(self, goal_handle):
        sequence = [0, 1]
        for _ in range(2, max(2, goal_handle.request.order)):
            sequence.append(sequence[-1] + sequence[-2])
            feedback = Fibonacci.Feedback()
            feedback.sequence = sequence.copy()
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        return result


def main(args=None):
    rclpy.init(args=args)
    node = BenchServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
