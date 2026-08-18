import time

import rclpy
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient
from rclpy.node import Node


class BenchClient(Node):
    def __init__(self):
        super().__init__("bench_client")
        for name, default in (
            ("n", 5),
            ("goals", 100),
            ("order", 20),
            ("repetitions", 1),
        ):
            self.declare_parameter(name, default)

        self.n = self.get_parameter("n").value
        self.goals = self.get_parameter("goals").value
        self.order = self.get_parameter("order").value
        self.repetitions = self.get_parameter("repetitions").value
        self.action_clients = [
            ActionClient(self, Fibonacci, f"/bench_action_{index}")
            for index in range(self.n)
        ]

    def ignore_feedback(self, _):
        pass

    def run(self):
        for client in self.action_clients:
            client.wait_for_server()

        for repetition in range(1, self.repetitions + 1):
            start = time.perf_counter()
            for _ in range(self.goals):
                for client in self.action_clients:
                    goal = Fibonacci.Goal()
                    goal.order = self.order
                    sent = client.send_goal_async(
                        goal, feedback_callback=self.ignore_feedback
                    )
                    rclpy.spin_until_future_complete(self, sent)
                    handle = sent.result()
                    if not handle.accepted:
                        raise RuntimeError("goal rejected")
                    result = handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result)

            duration = time.perf_counter() - start
            print(
                f"BENCH_RESULT n={self.n} goals_per_action={self.goals} "
                f"repetition={repetition} Duration={duration:.6f}",
                flush=True,
            )


def main(args=None):
    rclpy.init(args=args)
    node = BenchClient()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
