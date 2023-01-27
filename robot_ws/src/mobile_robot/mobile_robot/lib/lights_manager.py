#!/usr/bin/env python3

from time import sleep
from trilobot import *

SHOW_TIME = 1  # How long in seconds to have each pattern visible for
CLEAR_TIME = 0.1  # How long in seconds to have the underlights off between each pattern

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

class LightsManager():

  def __init__(self):
    self.tbot = Trilobot()
    self.colors = [ RED, GREEN, BLUE]
    self.lights = [
      LIGHT_FRONT_RIGHT,
      LIGHT_FRONT_LEFT,
      LIGHT_MIDDLE_LEFT,
      LIGHT_REAR_LEFT,
      LIGHT_REAR_RIGHT,
      LIGHT_MIDDLE_RIGHT ]

  def Clear(self):
    self.tbot.clear_underlighting()

  def SetLeft(self, color):
    self.tbot.set_underlights(LIGHTS_LEFT, color)

  def SetRight(self, color):
    self.tbot.set_underlights(LIGHTS_RIGHT, color)

  def SetFront(self, color):
    self.tbot.set_underlights(LIGHTS_FRONT, color)
  
  def SetMiddle(self, color):
    self.tbot.set_underlights(LIGHTS_MIDDLE, color)

  def SetRear(self, color):
    self.tbot.set_underlights(LIGHTS_REAR, color)

  def SetLeftDiagonal(self, color):
    self.tbot.set_underlights(LIGHTS_LEFT_DIAGONAL, color)

  def SetRightDiagonal(self, color):
    self.tbot.set_underlights(LIGHTS_RIGHT_DIAGONAL, color)

  def SetAll(self, color):
    self.tbot.fill_underlighting(color)

  def Set(self, light, color):
    self.tbot.set_underlight(light, color)

  def Test(self):
    self.Clear()
    sleep(CLEAR_TIME)
    for color in self.colors:
      for light in self.lights:
        self.Set(light, color)
        sleep(SHOW_TIME)
        self.Clear()
        sleep(CLEAR_TIME)
      self.SetAll(color)
      sleep(SHOW_TIME)
      self.Clear()
      sleep(CLEAR_TIME)

def main(args=None):
  lm = LightsManager()
  lm.Test()

if __name__ == '__main__':
  main()
