import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

# A publisher node for publishing a string message
class RePublisher(Node):
    def __init__(self):
        # initialize the node with the name 'republisher'
        super().__init__('republisher')

        # create a publisher object with name 'topic' and message type 'String'
        self.pub = self.create_publisher(String, 'topic', 10)

    def publish(self, msg):
        # create a message object of type 'String'
        msg = String()
        # assign the message to be published
        msg.data = 'Hello, World!'
        # publish the message
        self.pub.publish(msg)

# A subscriber node for subscribing to a string message,
# and forward the message to another topic
class SubscribeForward(Node):
    def __init__(self):
        # initialize the node with the name 'subscribe_forward'
        super().__init__('subscribe_forward')

        # create a subscriber object with name 'topic' and message type 'String'
        self.sub = self.create_subscription(String, 'topic', self.callback, 10)
        
        # allocate a space for temporarily storing the message
        self.msg = String()

        # the publisher object for republishing the message
        self.pub_object = RePublisher()

        # create a thread for publishing the message
        self.thread = threading.Thread(target=self.publish_thread)
        self.thread.start()

    def __del__(self):
        # destroy the thread
        self.thread.join()

    def callback(self, msg):
        # print the message received
        self.get_logger().info('I heard: "%s"' % msg.data)

        # store the message
        self.msg = msg

    def publish_thread(self):
        # publish the message
        self.pub_object.publish(self.msg)

        # wait for 0.05 seconds
        time.sleep(0.05)

def main(args=None):
    # initialize the ROS2 system
    rclpy.init(args=args)

    # create a subscriber node
    node = SubscribeForward()

    # keep the node running
    rclpy.spin(node)

    # destroy the node explicitly
    node.destroy_node()

    # shutdown the ROS2 system
    rclpy.shutdown()