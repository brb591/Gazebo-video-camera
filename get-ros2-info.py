import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class Ros2InfoNode(Node):

    def __init__(self):
        super().__init__('ros2_info_node')
    
    def get_topics(self):
        topics = self.get_topic_names_and_types(no_demangle=False)
        for topic in topics:
            print(topic)
    
    def get_namespaces(self):
        namespaces = self.get_node_names_and_namespaces()
        for ns in namespaces:
            print(ns)

def main(args=None):
    rclpy.init(args=args)

    ros2_info_node = Ros2InfoNode()

    print('ACTIVE TOPICS')
    ros2_info_node.get_topics()

    print('ACTIVE NAMESPACES')
    ros2_info_node.get_namespaces()

    ros2_info_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()