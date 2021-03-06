import math, time
import idm
from util import *

OUTER = 1
INNER = -1
VEH_WIDTH = SCALE # half of vehicle width
VEH_LENGTH = 2 * SCALE # half of vehicle height
WHE_WIDTH = 5 # half of wheel width
WHE_LENGTH = 15 # half of wheel length


class Vehicle:

  # Use (x,y) of center of the vehicle's bounding box
  # angle in radians
  def __init__(self, trackX, trackY, r, theta, direc, idNum):
    self.angSpeed = direc * toAngular(idm.OPT_VELOCITY, r) # deg/s
    self.trackX = trackX
    self.trackY = trackY
    self.r = r
    self.theta = theta # degrees
    self.looped = False
    vehX = r + self.trackX
    vehY = self.trackY
    self.direc = direc
    self.id = idNum
    self.pending = False
    self.passingIntersection = False
    self.inCriticalSection = False
    self.warnings = []
    self.inChain = False
    self.leadVehicle = None
    self.acceleration = 0
    self.optAngSpeed = direc * toAngular(idm.OPT_VELOCITY, r)
    self.canvas = []
    self.numCarsBehind = 0
    self.communicatedPresence = False

    self.totAngSpeed = 0
    self.totAngAcceleration = 0
    self.totAngDeceleration = 0
    self.totCntAcceleration = 0
    self.totCntDeceleration = 0
    self.total = 0
    self.waitingTime = 0

    self._createVehicle(vehX, vehY)


  def _createVehicle(self, vehX, vehY):
    self.vehPoints = [(vehX - VEH_WIDTH, vehY - VEH_LENGTH),
                        (vehX - VEH_WIDTH, vehY + VEH_LENGTH),
                        (vehX + VEH_WIDTH, vehY + VEH_LENGTH),
                        (vehX + VEH_WIDTH, vehY - VEH_LENGTH)]

    self.wheelPoints = []
    for x, y in self.vehPoints:
      wheel = []
      wheel.append((x - WHE_WIDTH, y - WHE_LENGTH))
      wheel.append((x - WHE_WIDTH, y + WHE_LENGTH))
      wheel.append((x + WHE_WIDTH, y + WHE_LENGTH))
      wheel.append((x + WHE_WIDTH, y - WHE_LENGTH))
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
    self.dirPoints = [(vehX, vehY - (self.direc * VEH_LENGTH//6)),
      (vehX - VEH_WIDTH //2, vehY + (self.direc * VEH_LENGTH//6)),
      (vehX + VEH_WIDTH //2, vehY + (self.direc * VEH_LENGTH//6))]
    self.dirRadii = self._getRadii(self.dirPoints)
    self.dirAngles = self._getAngles(self.dirPoints, self.dirRadii)

    self._initialTurn()

  def _initialTurn(self):
    newPoints = []
    for i in range(len(self.vehRadii)):
      self.vehAngles[i] += self.theta
      newPoints.append(self._xyCoord(self.vehRadii[i], self.vehAngles[i]))
    self.vehPoints = newPoints

    newWheels = []
    for i in range(len(self.wheelPoints)):
      indWheel = []
      for j in range(len(self.wheelPoints)):
        self.wheelAngles[i][j] += self.theta
        indWheel.append(self._xyCoord(self.wheelRadii[i][j], self.wheelAngles[i][j]))
      newWheels.append(indWheel)
    self.wheelPoints = newWheels

    newDir = []
    for i in range(len(self.dirRadii)):
      self.dirAngles[i] += self.theta
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

  def _passIntersection(self, time):
    prev = self.theta % MAX_DEG
    self.theta = (self.theta + (self.angSpeed * time)) % MAX_DEG
    curr = self.theta % MAX_DEG
    self.looped = (prev < (MAX_DEG // 2) and curr > (MAX_DEG // 2) and
      (not self.looped))


  def _turnCar(self, time):
    newPoints = []
    for i in range(len(self.vehRadii)):
        self.vehAngles[i] += (self.angSpeed * time)
        newPoints.append(self._xyCoord(self.vehRadii[i], self.vehAngles[i]))
    self.vehPoints = newPoints


  def _turnWheels(self, time):
    newWheels = []
    for i in range(len(self.wheelPoints)):
        indWheel = []
        for j in range(len(self.wheelPoints)):
          self.wheelAngles[i][j] += (self.angSpeed * time)
          indWheel.append(self._xyCoord(self.wheelRadii[i][j], self.wheelAngles[i][j]))
        newWheels.append(indWheel)
    self.wheelPoints = newWheels


  def _turnDirection(self, time):
    newDir = []
    for i in range(len(self.dirRadii)):
        self.dirAngles[i] += (self.angSpeed * time)
        newDir.append(self._xyCoord(self.dirRadii[i], self.dirAngles[i]))
    self.dirPoints = newDir

  def _updateAngSpeed(self):
    newSpeed = min(max(abs(self.angSpeed) + self.acceleration, 0), abs(self.optAngSpeed))
    self.angSpeed = self.direc * newSpeed
    if (self.angSpeed == self.optAngSpeed):
      self.acceleration = 0

  def _updateTotals(self, time):
    self.total += 1
    self.totAngSpeed += abs(self.angSpeed)
    if (self.acceleration > 0): 
      self.totAngAcceleration += self.acceleration
      self.totCntAcceleration += 1
    elif (self.acceleration < 0): 
      self.totAngDeceleration += abs(self.acceleration)
      self.totCntDeceleration += 1
    if (self.angSpeed == 0): self.waitingTime += (time * UPDATE_TIME)

  def _turn(self, time):
    self._turnCar(time)
    self._turnWheels(time)
    self._turnDirection(time)
    self._passIntersection(time)

  def update(self, time):
    self._updateAngSpeed()
    self._turn(time)
    self._updateTotals(time)

  def setNumCarsBehind(self, val):
    self.numCarsBehind = val

  def hasCommunicatedPresence(self):
    return self.communicatedPresence

  def setCommunicatedPresence(self, val):
    self.communicatedPresence = val

  def getVehAngles(self):
    return self.vehAngles

  # adds warning to vehicles list of warnings.
  # returns warnings the vehicles has not seen before
  def addWarnings(self, warnings):
    result = []
    for warning in warnings:
      if not warning in self.warnings:
        self.warnings.append(warning)
        result.append(warning)
    return result

  def handleWarning(self):
    pass

  def getID(self):
    return self.id

  def getRadius(self):
    return self.r

  def getTheta(self):
    return self.theta % MAX_DEG

  def isPassingIntersection(self):
    return self.passingIntersection

  def setPassingIntersection(self, passing=True):
    self.passingIntersection = passing

  def isInCriticalSection(self):
    return self.inCriticalSection

  def setInCriticalSection(self, val):
    self.inCriticalSection = val

  def setChain(self, val):
      self.inChain = val

  def isInChain(self):
      return self.inChain

  def setLeadVehicle(self, vehicle):
      self.leadVehicle = vehicle

  def getLeadVehicle(self):
      return self.leadVehicle

  def getX(self):
    return self.trackX + math.cos(toRadians(self.theta)) * self.r

  def getY(self):
    return self.trackY - math.sin(toRadians(self.theta)) * self.r

  def isPending(self):
    return self.pending

  def setPending(self, pending=True):
    self.pending = pending

  def getDirection(self):
    return self.direc

  def getAngSpeed(self):
    return self.angSpeed

  def setAngSpeed(self, speed):
    self.angSpeed = speed

  def getVehPoints(self):
    return self.vehPoints

  def getNumCarsBehind(self):
    return self.numCarsBehind

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

  def getCarAngle(self):
    r = (self.r * 1.0) - VEH_WIDTH - WHE_WIDTH
    return toDegrees((VEH_LENGTH * 2 + WHE_LENGTH * 2) / r)

  def getTrack(self):
    return (self.trackX, self.trackY)

  def getAcceleration(self):
    return self.acceleration

  # takes in acceleration as linear
  def setAcceleration(self, acceleration, angular=False):
    if angular:
      self.acceleration = acceleration
    else:
      self.acceleration = toAngular(acceleration, self.r)

  def increaseAngSpeed(self, amount):
    newSpeed = min(max(abs(self.angSpeed) + amount, 0), abs(self.optAngSpeed))
    self.angSpeed = self.direc * newSpeed

  def decreaseAngSpeed(self, amount):
    newSpeed = min(max(abs(self.angSpeed) - amount, 0), abs(self.optAngSpeed))
    self.angSpeed = self.direc * newSpeed

  def getAvgAngSpeed(self):
    if self.total == 0: return 0
    return (self.totAngSpeed / self.total)

  def getAvgAngAcceleration(self):
    if self.total == 0: return 0
    return (self.totAngAcceleration / self.total)

  def getAvgAngDeceleration(self):
    if self.total == 0: return 0
    return (self.totAngDeceleration / self.total)

  def getTotCntAcceleration(self):
    return self.totCntAcceleration

  def getTotCntDeceleration(self):
    return self.totCntDeceleration

  def getNetAcceleration(self):
    return ((self.totAngAcceleration - self.totAngDeceleration) / self.total)

  def getWaitingTime(self):
    return self.waitingTime

  def getLooped(self):
    return self.looped

  def getVehicleCanvas(self):
    return self.canvas[0]

  def getWheelsCanvas(self):
    return self.canvas[1]

  def getDirCanvas(self):
    return self.canvas[2]

  def getIDCanvas(self):
    return self.canvas[3]

  def getDetRadCanvas(self):
    return self.canvas[4]

  def setCanvas(self, canvas):
    self.canvas = canvas
