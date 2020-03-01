LEFT = 0
RIGHT = 1

class Vehicle:

  def __init__(self, x, y):
    self.speedLeft = 0
    self.speedRight = 0
    self.enableLeft = False
    self.enableRight = False
    self.x = x
    self.y = y
  
  def set_speed(self, side, speed):
    if (side == LEFT):
      self.speedLeft = speed
    elif (side == RIGHT):
      self.speedRight = speed
    else:
      print("Error: Invalid side argument")

  def set_enable(self, side, enable):
    if (side == LEFT):
      self.enableLeft = enable
    elif (side == RIGHT):
      self.enableRight = enable
    else:
      print("Error: Invalid side argument")

  def get_position(self):
    return (self.x, self.y)