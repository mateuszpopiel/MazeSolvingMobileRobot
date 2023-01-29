#!/bin/bash/env python

from time import sleep
from trilobot import Trilobot, BUTTON_B

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Bool, String, Int16

RED = (255, 0, 0)
YELLOW = (255, 255, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

MAX_DISTANCE = 20.0
SPEED = 0.5

NO_LINE = 0b11111
STRAIGHT_LINE = 0b11011
SLIGHTLY_RIGHT = 0b11101
SLIGHTLY_LEFT = 0b10111
TURN_LEFT = 0b00011
TURN_RIGHT = 0b11000

class MazeSolver(Node):

  def __init__(self):
    
    super().__init__('maze_solver')

    self.tbot = Trilobot()

    # # wall detection
    # self.distance = self.tbot.read_distance()
    # self.distance_msg = Float32()
    # timer_distance_period = 0.2 # seconds
    # self.pub_distance = self.create_publisher(Float32, 'distance', 10)
    # self.timer_distance = self.create_timer(
    #   timer_distance_period,
    #   self.timer_distance_callback)

    # line detection
    self.line = STRAIGHT_LINE
    self.prev_line = STRAIGHT_LINE
    self.end_of_road = False
    self.should_turn_left=False
    self.should_turn_right=False
    self.eor_cnt = 0
    self.line_sub = self.create_subscription(
      Int16, 'line', self.line_callback, 10)
    self.line_sub
    self.pub_activity = self.create_publisher(String, 'activity', 10)
    self.activity_msg = String()

    timer_run_period = 0.1 # seconds
    self.timer = self.create_timer(timer_run_period, self.timer_run_callback)

    self.get_logger().info('Maze solver is initialised.')

  def line_callback(self, msg):
    self.line = msg.data
    self.get_logger().info('Get line: {0:b}'.format(self.line))

  def timer_distance_callback(self):
    self.distance = self.tbot.read_distance()
    self.get_logger().info('Distance: %.2f' % self.distance)
    self.distance_msg.data = float(self.distance)
    self.pub_distance.publish(self.distance_msg)
  
  def timer_run_callback(self):
    line = self.line
    if self.should_turn_right:
      self.handle_turn_right()
    elif self.should_turn_left:
      self.handle_turn_left()
    else:
      # handle straight line
      if self.prev_line != NO_LINE:
          self.eor_cnt = 0
      if line == STRAIGHT_LINE:
        if self.end_of_road:
          self.end_of_road = False
        self.tbot.forward(SPEED)
        activity = "going forward"
      elif line == SLIGHTLY_RIGHT:
        self.tbot.turn_right(SPEED)
        activity = "turning right"
      elif line == SLIGHTLY_LEFT:
        self.tbot.turn_left(SPEED)
      elif line == TURN_LEFT:
        self.should_turn_left = True
      elif line == TURN_RIGHT:
        self.should_turn_right = True
        activity = "turning_left"
      elif line == NO_LINE and self.prev_line == NO_LINE:
        self.eor_cnt+=1
        if self.eor_cnt > 3:
          self.eor_cnt = 0
          self.end_of_road = True
          self.tbot.turn_right(SPEED)
          activity = "turning_right"
        else:
          activity = "EOR possible"
      else:
        activity = "don't know what is happenning o_O"
      self.prev_line = line

    self.get_logger().info('Activity: %s' % activity)
    self.activity_msg.data = activity
    self.pub_activity.publish(self.activity_msg)

  def handle_turn_right(self):
    self.tbot.curve_forward_right(SPEED)
    if self.line == STRAIGHT_LINE:
      self.should_turn_right = False

  def handle_turn_left(self):
    self.tbot.curve_forward_left(SPEED)
    if self.line == STRAIGHT_LINE:
      self.should_turn_left = False


def main(args=None):
  rclpy.init(args=args)
  ms = MazeSolver()
  rclpy.spin(ms)
  ms.kill()
  ms.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
