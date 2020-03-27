import math, time
# import path_planning

LEFT = 0
RIGHT = 1
VEH_WIDTH = 25 # half of vehicle width
VEH_LENGTH = 50 # half of vehicle height

class Vehicle:

  # Use (x,y) of center of the vehicle's bounding box
  # angle in radians
  def __init__(self, trackX, trackY, r, theta, turn, idNum):
    self.angSpeed = 1
    self.trackX = trackX
    self.trackY = trackY
    self.r = r
    self.theta = theta
    vehX = r + self.trackX
    vehY = self.trackY
    self.direc = turn
    self.id = idNum
    self.createVehicle(vehX, vehY)


  def createVehicle(self, vehX, vehY):
    self.vehPoints = [(vehX - VEH_WIDTH, vehY - VEH_LENGTH),
                        (vehX - VEH_WIDTH, vehY + VEH_LENGTH),
                        (vehX + VEH_WIDTH, vehY + VEH_LENGTH),
                        (vehX + VEH_WIDTH, vehY - VEH_LENGTH)]

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

    # triangle direction marker
    if (self.direc == LEFT): self.dirPoints = [(vehX, vehY - VEH_LENGTH//3)]
    else: self.dirPoints = [(vehX, vehY + VEH_LENGTH//3)]
    self.dirPoints.extend([(vehX - VEH_WIDTH //2, vehY),
                      (vehX + VEH_WIDTH //2, vehY)])
    self.dirRadii = self.getRadii(self.dirPoints)
    self.dirAngles = self.getAngles(self.dirPoints, self.dirRadii)

    self.initialTurn()

  def initialTurn(self):
    newPoints = []
    for i in range(len(self.vehRadii)):
      self.vehAngles[i] += (self.theta / (2 * math.pi)) * 360.0
      newPoints.append(self.xyCoord(self.vehRadii[i], self.vehAngles[i]))
    self.vehPoints = newPoints

    newWheels = []
    for i in range(len(self.wheelPoints)):
      indWheel = []
      for j in range(len(self.wheelPoints)):
        self.wheelAngles[i][j] += (self.theta / (2 * math.pi)) * 360.0
        indWheel.append(self.xyCoord
            (self.wheelRadii[i][j], self.wheelAngles[i][j]))
      newWheels.append(indWheel)
    self.wheelPoints = newWheels

    newDir = []
    for i in range(len(self.dirRadii)):
      self.dirAngles[i] += (self.theta / (2 * math.pi)) * 360.0
      newDir.append(self.xyCoord(self.dirRadii[i], self.dirAngles[i]))
    self.dirPoints = newDir

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


  def xyCoord(self, r, deg):
    a = 2 * math.pi * (deg / 360.0)
    x = self.trackX + r * math.cos(a)
    y = self.trackY - r * math.sin(a)
    return (x, y)


  def turnCar(self):
    newPoints = []
    for i in range(len(self.vehRadii)):
        if (self.direc == LEFT): self.vehAngles[i] += self.angSpeed
        else: self.vehAngles[i] -= self.angSpeed
        newPoints.append(self.xyCoord(self.vehRadii[i], self.vehAngles[i]))
    self.vehPoints = newPoints


  def turnWheels(self):
    newWheels = []
    for i in range(len(self.wheelPoints)):
        indWheel = []
        for j in range(len(self.wheelPoints)):
          if (self.direc == LEFT): self.wheelAngles[i][j] += self.angSpeed
          else: self.wheelAngles[i][j] -= self.angSpeed
          indWheel.append(self.xyCoord
            (self.wheelRadii[i][j], self.wheelAngles[i][j]))
        newWheels.append(indWheel)
    self.wheelPoints = newWheels


  def turnDirection(self):
    newDir = []
    for i in range(len(self.dirRadii)):
        if (self.direc == LEFT): self.dirAngles[i] += self.angSpeed
        else: self.dirAngles[i] -= self.angSpeed
        newDir.append(self.xyCoord(self.dirRadii[i], self.dirAngles[i]))
    self.dirPoints = newDir


  def turn(self):
    # print(self.vehPoints)

    self.turnCar()
    self.turnWheels()
    self.turnDirection()

    self.theta = (self.theta +
        (2 * math.pi * (self.angSpeed / 360.0))) % (2 * math.pi)

  def getID(self):
    return self.id

  def getTheta(self):
    return self.theta

  def getDirection(self):
    return self.direc

  def increaseAngSpeed(self, speed):
    self.angSpeed = min(self.angSpeed + speed, 5)
    #need to change after finding relationship between pixels and m/s

  def decreaseAngSpeed(self, speed):
    self.angSpeed = max(self.angSpeed - speed, 0)

  def setAngSpeed(self, speed):
    self.angSpeed = speed

  def getAngSpeed(self):
    return self.angSpeed

  def getVehPoints(self):
    return self.vehPoints

  def getWheelPoints(self):
    return self.wheelPoints

  def getDirPoints(self):
    return self.dirPoints
