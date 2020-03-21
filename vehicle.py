import math

LEFT = 0
RIGHT = 1
VEH_WIDTH = 30
VEH_LENGTH = 60

class Vehicle:

  # Use (x,y) of center of the vehicle's bounding box
  # angle in radians
  def __init__(self, trackX, trackY, r, theta):
    self.speed = 2
    self.trackX = trackX
    self.trackY = trackY
    self.r = r
    self.theta = theta
    vehX = r * math.cos(theta) + self.trackX
    vehY = r * math.sin(theta) + self.trackY
    self.createVehicle(vehX, vehY, VEH_WIDTH, VEH_LENGTH)


  def createVehicle(self, x, y, width, length):
      self.vehPoints = [(x - width, y - length),
                          (x - width, y + length),
                          (x + width, y + length),
                          (x + width, y - length)]

      carWheels = (-5, -15, 5, 15)
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

      # list of 4 radii per wheel
      self.wheelRadii = []
      for w in self.wheelPoints:
        self.wheelRadii.append(self.getRadii(w))
      # list of 4 angles per wheel
      self.wheelAngles = []
      for i in range(len(self.wheelPoints)):
        self.wheelAngles.append(self.getAngles
          (self.wheelPoints[i], self.wheelRadii[i]))

      self.initialTurn()

  def initialTurn(self):
      newPoints = []
      for i in range(len(self.vehRadii)):
        r = self.vehRadii[i]
        self.vehAngles[i] += (self.theta / (2 * math.pi)) * 360.0
        a = 2 * math.pi * (self.vehAngles[i] / 360.0)
        x = self.trackX + r * math.cos(a)
        y = self.trackY - r * math.sin(a)
        newPoints.append((x, y))
      self.vehPoints = newPoints

      newWheels = []
      for i in range(len(self.wheelPoints)):
        indWheel = []
        for j in range(len(self.wheelPoints)):
          r = self.wheelRadii[i][j]
          self.wheelAngles[i][j] += (self.theta / (2 * math.pi)) * 360.0
          a = 2 * math.pi * (self.wheelAngles[i][j] / 360.0)
          x = self.trackX + r * math.cos(a)
          y = self.trackY - r * math.sin(a)
          indWheel.append((x, y))
        newWheels.append(indWheel)
      self.wheelPoints = newWheels

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
    for i in range(self.speed):
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
        indWheel = []
        for j in range(len(self.wheelPoints)):
          r = self.wheelRadii[i][j]
          self.wheelAngles[i][j] += self.speed
          a = 2 * math.pi * (self.wheelAngles[i][j] / 360.0)
          x = self.trackX + r * math.cos(a)
          y = self.trackY - r * math.sin(a)
          indWheel.append((x, y))
        newWheels.append(indWheel)
      self.wheelPoints = newWheels


  def setSpeed(self, speed):
    self.speed = speed


  def getVehPoints(self):
    return self.vehPoints

  def getWheelPoints(self):
    return self.wheelPoints
