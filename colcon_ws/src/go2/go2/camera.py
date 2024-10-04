import cv2
import numpy as np
import rclpy
import rclpy.logging
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient


class Camera(Node):

    def __init__(self):
        super().__init__("camera_recorder", namespace="go2")
        ChannelFactoryInitialize(0)
        self._client = VideoClient()
        self._client.SetTimeout(3)
        self._client.Init()

        self._bridge = CvBridge()
        self._publisher = self.create_publisher(Image, "camera/image", 10)
        self._timer = self.create_timer(1, self._publish)

    def _publish(self):
        # Get Go2 front camera image.
        code, data = self._client.GetImageSample()

        # Convert image to OpenCV image.
        image_data = np.frombuffer(bytes(data), dtype=np.uint8)
        cv_image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

        # Convert OpenCV image to ROS image.
        ros_image = self._bridge.cv2_to_imgmsg(np.array(cv_image), encoding="bgr8")

        self._publisher.publish(ros_image)
        self.get_logger().info("Publishing image")


def main(args=None):
    rclpy.init(args=args)
    node = Camera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
