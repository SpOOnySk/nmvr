import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import sys
import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic',10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        input_text = input()
        try:
            action, coord, param = input_text.split(';')
            if action == "edit":
                msg = String()
                msg.data = input_text
                print("sending " + input_text)
                self.publisher.publish(msg)
                # self.get_logger().info('Publishing: "%s"' %msg.data)
                self.i +=1
            elif action == "quit":
                msg = String()
                msg.data = input_text
                print("sending " + input_text)
                self.publisher.publish(msg)
                time.sleep(1)
                sys.exit()
            else:
                print("Unable to send command. Unknow command.")
        except ValueError:
            print("Missing action, coord or params in message")


def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # run = True
    # while run:
    #     input_text = input()
    #     if input_text == 'quit':
    #         run = False
    #     elif len(input_text)>0:
    #         minimal_publisher.send_message(input_text)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()