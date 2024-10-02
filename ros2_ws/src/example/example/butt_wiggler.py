import rclpy
import rclpy.logging
from rclpy.node import Node
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient


class ButtWiggler(Node):

    def __init__(self):
        super().__init__("butt_wiggler")
        ChannelFactoryInitialize(0)
        self._client = SportClient()
        self._client.SetTimeout(10)
        self._client.Init()

        self._timer = self.create_timer(5, self._wiggle_butt)

    def _wiggle_butt(self):
        self.get_logger().info("Wiggling butt")
        self._client.WiggleHips()


def main(args=None):
    rclpy.init(args=args)
    node = ButtWiggler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
