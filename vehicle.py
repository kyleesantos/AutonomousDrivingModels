import math

LEFT = 0
RIGHT = 1

class Vehicle:

  # Use (x,y) of center of the vehicle's bounding box
  # angle in radians
  def __init__(self, x, y, theta):
    self.speedLeft = 0
    self.speedRight = 0
    self.enableLeft = False
    self.enableRight = False
    self.x = x
    self.y = y
    self.theta = theta
  

  def set_speed(self, speeds):
    self.speedLeft = speeds[0]
    self.speedRight = speeds[1]

    #defining angle as pure ratio of speeds
    if (self.speedRight > self.speedLeft):
      self.theta += self.speedRight / self.speedLeft
    else:
      self.theta -= self.speedLeft / self.speedRight


  def set_enable(self, side, enable):
    if (side == LEFT):
      self.enableLeft = enable
    elif (side == RIGHT):
      self.enableRight = enable
    else:
      print("Error: Invalid side argument")


  def get_position(self):
    return (self.x, self.y)


  def set_path(self, path):
    self.center_x = path[0]
    self.center_y = path[1]
    self.radius = path[2]


  def update_position(self):
    #relate angular speed and position to get next position
    if (self.speedRight > self.speedLeft):
      self.theta += self.speedRight / self.speedLeft
    else:
      self.theta -= self.speedLeft / self.speedRight
    self.x = (math.cos(self.theta) * self.radius) + self.center_x
    self.y = (math.sin(self.theta) * self.radius) + self.center_y