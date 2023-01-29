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
SLIGHTLY_LEFT = 0b11101
SLIGHTLY_RIGHT = 0b10111
TURN_LEFT = 0b00011
TURN_RIGHT = 0b11000
INTERSECTION = 0b00000
SPECIAL_SPOT = 0b10001

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
    self.left_turn_detected=False
    self.should_turn_right=False
    self.win_possible = False
    self.eor_cnt = 0
    self.win_cnt = 0
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
    # Right Wall Following Algorithm
    if self.win_possible:
      activity = self.handle_win()
    elif self.should_turn_right:
      activity = self.handle_turn_right()
    elif self.should_turn_left:
      activity = self.handle_turn_left()
    else:
      activity = self.handle_straight_line()

    self.get_logger().info('Activity: %s' % activity)
    self.activity_msg.data = activity
    self.pub_activity.publish(self.activity_msg)

  def handle_turn_right(self):
    self.tbot.curve_forward_right(SPEED)
    if self.line == STRAIGHT_LINE:
      self.should_turn_right = False
    return "turning left (turn detected)"

  def handle_turn_left(self):
    self.tbot.curve_forward_left(SPEED)
    if self.line == STRAIGHT_LINE:
      self.should_turn_left = False
    return "turning left (turn_detected)"

  def handle_straight_line(self):
    if self.prev_line != NO_LINE:
        self.eor_cnt = 0
    if self.line == STRAIGHT_LINE:
      if self.end_of_road:
        self.end_of_road = False
      self.tbot.forward(SPEED)
      activity = "going forward"
    elif self.line == SLIGHTLY_RIGHT:
      self.tbot.turn_left(SPEED)
      activity = "turning slightly left"
    elif self.line == SLIGHTLY_LEFT:
      self.tbot.turn_right(SPEED)
      activity = "turning slightly right"
    elif self.line == TURN_LEFT:
      self.left_turn_detected = True
      activity = "left turn detected"
    elif self.line == TURN_RIGHT:
      self.should_turn_right = True
      activity = "should turn right"
    elif self.line == INTERSECTION:
      self.should_turn_right = True
      activity = "should turn right on intersection"
    elif self.line == SPECIAL_SPOT:
      self.win_possible = True
      activity = "Win possible"
    elif self.line == NO_LINE and self.prev_line == NO_LINE:
      if self.left_turn_detected:
        self.should_turn_left = True
        self.left_turn_detected = False
        activity = "should turn left"
      else:
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
    self.prev_line = self.line
    return activity

  def handle_win(self):
    self.win_cnt += 1
    if self.win_cnt > 1:
      self.tbot.stop()
      self.tbot.fill_underlighting(BLUE)
      return "Maze solved!!!"
    else:
      if self.line != SPECIAL_SPOT:
        self.win_cnt = 0
      return "Didn't win yet"

def main(args=None):
  rclpy.init(args=args)
  ms = MazeSolver()
  rclpy.spin(ms)
  ms.kill()
  ms.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
