import math, time
import idm
from util import *

LEFT = -1
RIGHT = 1
VEH_WIDTH = 25 # half of vehicle width
VEH_LENGTH = 50 # half of vehicle height

class Vehicle:

  # Use (x,y) of center of the vehicle's bounding box
  # angle in radians
  def __init__(self, trackX, trackY, r, theta, direc, idNum):
    self.angSpeed = -1 if direc == LEFT else RIGHT     # deg/s
    self.trackX = trackX
    self.trackY = trackY
    self.r = r
    self.theta = theta
    vehX = r + self.trackX
    vehY = self.trackY
    self.direc = direc
    self.id = idNum
    self.pending = False
    self.acceleration = 0
    self.optAngSpeed = direc * toAngular(idm.OPT_VELOCITY, r)
    self._createVehicle(vehX, vehY)


  def _createVehicle(self, vehX, vehY):
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

    self.vehRadii = self._getRadii(self.vehPoints)
    self.vehAngles = self._getAngles(self.vehPoints, self.vehRadii)

    # list of 4 radii per wheel
    self.wheelRadii = []
    for w in self.wheelPoints:
      self.wheelRadii.append(self._getRadii(w))
    # list of 4 angles per wheel
    self.wheelAngles = []
    for i in range(len(self.wheelPoints)):
      self.wheelAngles.append(self._getAngles(self.wheelPoints[i], self.wheelRadii[i]))

    # triangle direction marker
    self.dirPoints = [(vehX, vehY - (self.direc * VEH_LENGTH//3))]
    self.dirPoints.extend([(vehX - VEH_WIDTH //2, vehY), (vehX + VEH_WIDTH //2, vehY)])
    self.dirRadii = self._getRadii(self.dirPoints)
    self.dirAngles = self._getAngles(self.dirPoints, self.dirRadii)

    self._initialTurn()

  def _initialTurn(self):
    newPoints = []
    for i in range(len(self.vehRadii)):
      self.vehAngles[i] += toDegrees(self.theta)
      newPoints.append(self._xyCoord(self.vehRadii[i], self.vehAngles[i]))
    self.vehPoints = newPoints

    newWheels = []
    for i in range(len(self.wheelPoints)):
      indWheel = []
      for j in range(len(self.wheelPoints)):
        self.wheelAngles[i][j] += toDegrees(self.theta)
        indWheel.append(self._xyCoord(self.wheelRadii[i][j], self.wheelAngles[i][j]))
      newWheels.append(indWheel)
    self.wheelPoints = newWheels

    newDir = []
    for i in range(len(self.dirRadii)):
      self.dirAngles[i] += toDegrees(self.theta)
      newDir.append(self._xyCoord(self.dirRadii[i], self.dirAngles[i]))
    self.dirPoints = newDir

  # Find the angle of each corner of the vehicle's bounding box
  def _getRadii(self, points):
    radii = []
    for x, y in points:
      radii.append(math.sqrt((self.trackX - x)**2 + (self.trackY - y)**2))
    return radii


  # Find the angle of each corner of the vehicle's bounding box
  def _getAngles(self, points, radii):
    angles = []
    for i in range(len(points)):
      x, y = points[i]
      x -= self.trackX
      y -= self.trackY
      r = radii[i]
      a = toDegrees(math.acos(float(x / r)))
      if (y > 0): a *= -1.0
      angles.append(a)
    return angles


  def _xyCoord(self, r, deg):
    a = toRadians(deg)
    x = self.trackX + r * math.cos(a)
    y = self.trackY - r * math.sin(a)
    return (x, y)


  def _turnCar(self):
    newPoints = []
    for i in range(len(self.vehRadii)):
        self.vehAngles[i] += self.angSpeed
        newPoints.append(self._xyCoord(self.vehRadii[i], self.vehAngles[i]))
    self.vehPoints = newPoints


  def _turnWheels(self):
    newWheels = []
    for i in range(len(self.wheelPoints)):
        indWheel = []
        for j in range(len(self.wheelPoints)):
          self.wheelAngles[i][j] += self.angSpeed
          indWheel.append(self._xyCoord(self.wheelRadii[i][j], self.wheelAngles[i][j]))
        newWheels.append(indWheel)
    self.wheelPoints = newWheels


  def _turnDirection(self):
    newDir = []
    for i in range(len(self.dirRadii)):
        self.dirAngles[i] += self.angSpeed
        newDir.append(self._xyCoord(self.dirRadii[i], self.dirAngles[i]))
    self.dirPoints = newDir

  def _updateAngSpeed(self):
    if (self.direc == LEFT):
      self.angSpeed = max(min(self.angSpeed + self.acceleration, 0), self.optAngSpeed)
    else:
      self.angSpeed = min(max(self.angSpeed + self.acceleration, 0), self.optAngSpeed)
    
    if (self.angSpeed == self.optAngSpeed):
      self.optAngSpeed = 0

  def _turn(self):
    self._turnCar()
    self._turnWheels()
    self._turnDirection()

    self.theta = self.theta + toRadians(self.angSpeed) % (2 * math.pi)

  def update(self):
    self._updateAngSpeed()
    self._turn()

  def getID(self):
    return self.id

  def getRadius(self):
    return self.r

  def getTheta(self):
    return self.theta
  
  def getX(self):
    return self.trackX + math.cos(self.theta) * self.r
  
  def getY(self):
    return self.trackY - math.sin(self.theta) * self.r

  def isPending(self):
    return self.pending

  def setPending(self, pending=True):
    self.pending = pending

  def getDirection(self):
    return self.direc

  def setAngSpeed(self, speed):
    self.angSpeed = speed

  def getAngSpeed(self):
    return self.angSpeed

  def getVehPoints(self):
    return self.vehPoints

  # returns center point of vehicle
  def getPos(self):
    vehPoints = self.getVehPoints()
    sumx = sum([pos[0] for pos in vehPoints])
    sumy = sum([pos[1] for pos in vehPoints])
    return (sumx/len(vehPoints), sumy/len(vehPoints))

  def getWheelPoints(self):
    return self.wheelPoints

  def getDirPoints(self):
    return self.dirPoints

  def getAcceleration(self):
    return self.acceleration

  # takes in acceleration as linear
  def setAcceleration(self, acceleration):
    self.acceleration = toAngular(acceleration, self.r)

  def increaseAngSpeed(self, amount):
    if (self.direc == LEFT):
      self.angSpeed = max(min(self.angSpeed - amount, 0), self.optAngSpeed)
    else:
      self.angSpeed = min(max(self.angSpeed + amount, 0), self.optAngSpeed)
    
    print(self.angSpeed)

  def decreaseAngSpeed(self, amount):
    if (self.direc == LEFT):
      self.angSpeed = max(min(self.angSpeed + amount, 0), self.optAngSpeed)
    else:
      self.angSpeed = min(max(self.angSpeed - amount, 0), self.optAngSpeed)