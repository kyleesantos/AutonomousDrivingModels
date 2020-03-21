import math

LEFT = 0
RIGHT = 1

class Vehicle:

  # Use (x,y) of center of the vehicle's bounding box
  # angle in radians
  def __init__(self, x, y, r, theta):
    # self.speedLeft = 0
    # self.speedRight = 0
    # self.enableLeft = False
    # self.enableRight = False
    # self.x = x
    # self.y = y

    self.speed = 1
    self.count = 0
    self.trackX = x
    self.trackY = y
    self.r = r
    self.theta = theta
    self.x = r * math.cos(theta) + self.trackX
    self.y = r * math.sin(theta) + self.trackY
    vehWidth = 30
    vehLength = 60

    self.vehPoints = [(self.x - vehWidth, self.y - vehLength), 
                    (self.x - vehWidth, self.y + vehLength),
                    (self.x + vehWidth, self.y + vehLength),
                    (self.x + vehWidth, self.y - vehLength)]

    carWheels = (-10, -20, 10, 20)
    self.wheelPoints = []
    for x, y in self.vehPoints:
      wheel = []
      wheel.append((x + carWheels[0], y + carWheels[1]))
      wheel.append((x + carWheels[0], y + carWheels[3]))
      wheel.append((x + carWheels[2], y + carWheels[3]))
      wheel.append((x + carWheels[2], y + carWheels[1]))
      self.wheelPoints.append(wheel)


    self.vehRadii = self.getRadii(self.vehPoints)
    self.vehAngles = self.getAngles(self.vehPoints, self.vehRadii)

    self.wheelRadii = []
    for w in self.wheelPoints:
      self.wheelRadii.append(self.getRadii(w))
    self.wheelAngles = []
    for i in range(len(self.wheelPoints)):
      self.wheelAngles.append(self.getAngles
        (self.wheelPoints[i], self.wheelRadii[i]))


  # Find the angle of each corner of the vehicle's bounding box
  def getRadii(self, points):
    radii = []
    for x, y in points:
      radii.append(math.sqrt((self.trackX - x)**2 + (self.trackY - y)**2))
    return radii


  # Find the angle of each corner of the vehicle's bounding box
  def getAngles(self, points, radii):
    angles = []
    for i in range(len(points)):
      x, y = points[i]
      x -= self.trackX
      y -= self.trackY
      r = radii[i]
      a = math.acos(float(x / r)) / (2 * math.pi) * 360.0
      if (y > 0): a *= -1.0
      angles.append(a)
    return angles


  def turnLeft(self):
    self.count += 1
    if (self.count == self.speed):
      newPoints = []
      for i in range(len(self.vehRadii)):
        r = self.vehRadii[i]
        self.vehAngles[i] += self.speed
        a = 2 * math.pi * (self.vehAngles[i] / 360.0)
        x = self.trackX + r * math.cos(a)
        y = self.trackY - r * math.sin(a)
        newPoints.append((x, y))
      self.vehPoints = newPoints

      newWheels = []
      for i in range(len(self.wheelPoints)):
        for j in range(len(self.wheelPoints)):
          r = self.wheelRadii[i][j]
          self.wheelAngles[i][j] += self.speed
          a = 2 * math.pi * (self.wheelAngles[i][j] / 360.0)
          x = self.trackX + r * math.cos(a)
          y = self.trackY - r * math.sin(a)
          newWheels.append((x, y))
      self.wheelPoints = newWheels

      self.count = 0


  def setSpeed(self, speed):
    self.speed = speed


  def getVehPoints(self):
    return self.vehPoints

  def getWheelPoints(self):
    return self.wheelPoints

  # Pre SB
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