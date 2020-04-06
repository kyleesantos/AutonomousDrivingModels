import math
import vehicle
from util import *

# Intelligent Driver Model Parameters
REAL_OPT_VELOCITY = 0.3  # m/s
REAL_MAX_ACCEL = 0.1     # m/s2
REAL_OPT_DECEL = 0.1     # m/s2
REAL_BUFFER_DIST = 6.5   # m
REAL_DETECTION_DIST = 6  # m

OPT_VELOCITY = SCALE * REAL_OPT_VELOCITY  # pixels/s
MAX_ACCEL = SCALE * REAL_MAX_ACCEL        # pixels/s2
OPT_DECEL = SCALE * REAL_OPT_DECEL        # pixels/s2
BUFFER_DIST = SCALE * REAL_BUFFER_DIST    # pixels

TIME_HEADWAY = 1.5    # s
ACCEL_EXP = 4

# Information Constraints Parameters
DETECTION_DIST = SCALE * REAL_DETECTION_DIST  # pixels


def calculateAccel(dist, v, delta_v):
  a = math.pow((v/OPT_VELOCITY), ACCEL_EXP)
  min_gap = BUFFER_DIST + TIME_HEADWAY*v + (v*delta_v/(2*math.sqrt(MAX_ACCEL*OPT_DECEL)))
  b = math.pow((min_gap / dist),2)
  accel = MAX_ACCEL*(1 - a - b)
  return accel


# returns true if vehicle2 is within the information constraint
def isDetectable(vehicle1, vehicle2):
  x1,y1 = vehicle1.getX(), vehicle1.getY()
  x2,y2 = vehicle2.getX(), vehicle2.getY()
  return ((vehicle1 != vehicle2) and
          (math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2) <= math.pow(DETECTION_DIST, 2)))

def isOnPath(vehicle1, vehicle2):
  return vehicle1.getDirection() == vehicle2.getDirection()


def findNearestOnPath(vehicle, vehicles):
  nearby_vehicles = []
  for vehicle2 in vehicles:
    if isOnPath(vehicle, vehicle2):
      nearby_vehicles.append(vehicle2)
  return nearby_vehicles


def findClosestObstacleAhead(vehicle1, vehicles):
  closest_diff = MAX_DEG
  closest_speed = 0
  theta1 = vehicle1.getTheta()
  thetas = [veh.getTheta() for veh in vehicles]
  if not vehicle1.isPassingIntersection():
    if vehicle1.getDirection() == CTR_CLK:
      thetas.append(RIGHT_ENTRANCE_THETA + toAngular(BUFFER_DIST/2,vehicle1.getRadius()))
    else:
      thetas.append(LEFT_ENTRANCE_THETA - toAngular(BUFFER_DIST/2,vehicle1.getRadius()))

  for (i,theta2) in enumerate(thetas):
    if vehicle1.direc == CLK:
      diff = theta1 - theta2 if theta1 > theta2 else theta1 + MAX_DEG - theta2
    else:
      diff = theta2 - theta1 if theta2 > theta1 else theta2 + MAX_DEG - theta1
    if diff < closest_diff:
      closest_diff = diff
      if i < len(vehicles):
        closest_speed = vehicles[i].getAngSpeed()
      else:
        closest_speed = 0

  return closest_speed, closest_diff


def updateAccels(vehicles):
  for vehicle1 in vehicles:
    # enforce information constraints
    nearby_vehicles = findNearestOnPath(vehicle1, vehicles)
    (closest_speed,diff) = findClosestObstacleAhead(vehicle1, nearby_vehicles)
    if diff < MAX_DEG:
      dist = MAX_RAD * vehicle1.getRadius() * (diff / MAX_DEG)
      lin_speed1 = abs(toLinear(vehicle1.getAngSpeed(), vehicle1.getRadius()))
      lin_speed2 = abs(toLinear(closest_speed, vehicle1.getRadius()))
      new_accel = calculateAccel(dist, lin_speed1, lin_speed1 - lin_speed2)
      vehicle1.setAcceleration(new_accel)
    else:
      vehicle1.setAcceleration(MAX_ACCEL)
