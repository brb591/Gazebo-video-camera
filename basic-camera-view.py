import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2


class ImgSubscriber(Node):
    def __init__(self, topic_name):
        # img_subscriber is the name of the node
        super().__init__('img_subscriber')

        self.subscription = self.create_subscription(Image,
            topic_name,
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription

        # This bridges ROS2 and OpenCV
        self.br = CvBridge()

    def listener_callback(self, data):
        # Convert the image to a cv2 image
        current_frame = self.br.imgmsg_to_cv2(data)
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)


def main(args=None):
    topic = sys.argv[1]

    rclpy.init(args=args)
    
    img_subscriber = ImgSubscriber(topic)

    rclpy.spin(img_subscriber)

    img_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
