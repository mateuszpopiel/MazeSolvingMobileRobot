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
SLIGHTLY_LEFT_2 = 0b11001
SLIGHTLY_RIGHT = 0b10111
SLIGHTLY_RIGHT_2 = 0b10011
TURN_LEFT = 0b00011
TURN_LEFT_2 = 0b00001
TURN_RIGHT = 0b11000
TURN_RIGHT_2 = 0b10000
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
    self.left_turn_possible=False
    self.left_turn_detected = False
    self.should_turn_right=False
    self.win_possible = False
    self.eor_cnt = 0
    self.win_cnt = 0
    self.left_cnt = 0
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
    elif self.left_turn_possible or self.left_turn_detected:
      activity = self.handle_turn_left()
    else:
      activity = self.handle_straight_line()

    self.prev_line = self.line
    self.get_logger().info('Activity: %s' % activity)
    self.activity_msg.data = activity
    self.pub_activity.publish(self.activity_msg)

  def handle_turn_right(self):
    if self.line == SPECIAL_SPOT:
      self.should_turn_right = False
      self.win_possible = True
      activity = "possible win on right turn"
    else:
      self.tbot.curve_forward_right(SPEED)
      if self.line == STRAIGHT_LINE:
        self.tbot.forward(SPEED)
        self.should_turn_right = False
      activity = "turning right"
    return activity

  def handle_turn_left(self):
    if self.line == SPECIAL_SPOT:
      self.should_turn_left = False
      self.win_possible = True
      activity = "possible win on left turn"
    else:
      self.left_cnt += 1
      if self.left_cnt > 1:
        self.left_turn_detected = True
        self.left_turn_possible = False
        if self.left_turn_detected:
          if self.line == STRAIGHT_LINE:
            self.tbot.forward(SPEED)
            self.left_turn_detected = False
            activity = "turned left"
            self.left_cnt = 0
          else:
            self.tbot.curve_forward_left(SPEED)
            activity = "turning left"
      # elif self.line == TURN_LEFT or self.line == TURN_LEFT_2:
      elif self.line == TURN_LEFT:
        activity = "left turn still possible"
      else:
        activity = "no turn detected"
        self.left_turn_possible = False
    return activity

  def handle_win(self):
    self.win_cnt += 1
    SPEED = 0.3
    if self.win_cnt > 1:
      self.tbot.stop()
      self.tbot.fill_underlighting(BLUE)
      activity = "Maze solved!!!"
    else:
      if not self.possible_winning_spot():
        self.win_cnt = 0
        self.win_possible = False
        SPEED = 0.5
        activity = "Didn't win yet"
      else:
        activity = "Win still possible"
    return activity

  def handle_straight_line(self):
    if self.prev_line != NO_LINE:
        self.eor_cnt = 0
    if self.line == STRAIGHT_LINE:
      if self.end_of_road:
        self.end_of_road = False
      self.tbot.forward(SPEED)
      activity = "going forward"
    elif self.line == SLIGHTLY_RIGHT or self.line == SLIGHTLY_RIGHT_2:
      self.tbot.curve_forward_left(SPEED)
      activity = "turning slightly left"
    elif self.line == SLIGHTLY_LEFT or self.line == SLIGHTLY_LEFT_2:
      self.tbot.curve_forward_right(SPEED)
      activity = "turning slightly right"
    elif self.line == INTERSECTION:
      self.should_turn_right = True
      activity = "should turn right on intersection"
    elif self.line == TURN_RIGHT or self.line == TURN_RIGHT_2:
    # elif self.line == TURN_RIGHT:
      self.should_turn_right = True
      activity = "should turn right"
    elif self.line == TURN_LEFT or self.line == TURN_LEFT_2:
    # elif self.line == TURN_LEFT:
      self.left_turn_possible = True
      activity = "left turn detected"
    elif self.line == SPECIAL_SPOT:
      self.win_possible = True
      activity = "Win possible"
    elif self.line == NO_LINE and self.prev_line == NO_LINE:
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
    return activity

  def possible_winning_spot(self):
    return (self.line == SPECIAL_SPOT) or \
      (self.line == TURN_LEFT_2) or \
      (self.line == TURN_RIGHT_2)


def main(args=None):
  rclpy.init(args=args)
  ms = MazeSolver()
  rclpy.spin(ms)
  ms.kill()
  ms.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
