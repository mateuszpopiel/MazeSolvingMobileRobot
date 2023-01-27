#!/bin/bash/env python

from time import sleep

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16, Bool
from mobile_robot.lib.line_detector import LineDetector

class LineDetectorNode(Node):

  def __init__(self):

    super().__init__('line_detector')

    self.publisher_ = self.create_publisher(Int16, 'line', 10)
    timer_period = 0.1 # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    
    self.line_detector = LineDetector()
    self.memory = 3
    self.line_measurements = [ [ 1 for i in range(5) ] for j in range(self.memory) ]

    self.lives=True
    self.get_logger().info('Line detector initialised.')

  def timer_callback(self):
    readings = self.line_detector.read_digital()
    msg = Int16()
    msg.data = 0
    n = len(readings)
    for i in range(n):
      msg.data += readings[i] << n-(i+1)
    self.publisher_.publish(msg)
    self.get_logger().info("Average readings: {0:b}".format(msg.data))

    analog = self.line_detector.read_analog()
    analog_str = '|'
    for i in range(len(analog)):
      analog_str += ' %d |' % analog[i]
    self.get_logger().info('Analog: "%s"' % analog_str)


def main(args=None):
  rclpy.init(args=args)
  ld = LineDetectorNode()
  rclpy.spin(ld)
  ld.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
