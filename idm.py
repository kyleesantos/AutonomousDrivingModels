import math
from vehicle import *
from util import *

SCALE = 25 # pixels/m

# Intelligent Driver Model Parameters
REAL_OPT_VELOCITY = 0.3  # m/s
REAL_MAX_ACCEL = 0.1     # m/s2
REAL_OPT_DECEL = 0.1     # m/s2
REAL_BUFFER_DIST = 6.5   # m
REAL_DETECTION_DIST = 3  # m

OPT_VELOCITY = SCALE * REAL_OPT_VELOCITY  # pixels/s
MAX_ACCEL = SCALE * REAL_MAX_ACCEL        # pixels/s2
OPT_DECEL = SCALE * REAL_OPT_DECEL        # pixels/s2
BUFFER_DIST = SCALE * REAL_BUFFER_DIST    # pixels

TIME_HEADWAY = 2    # s
ACCEL_EXP = 4

# Information Constraints Parameters
DETECTION_DIST = SCALE * REAL_DETECTION_DIST  # m


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


def findClosestVehicleAhead(vehicle1, vehicles):
  closest_diff = 360.0
  closest_vehicle = None
  theta1 = vehicle1.getTheta()
  for vehicle2 in vehicles:
    theta2 = vehicle2.getTheta()
    diff = theta2 - theta1 if theta2 > theta1 else theta2 + 360.0 - theta1
    if diff < closest_diff:
      closest_diff = diff
      closest_vehicle = vehicle2
  return closest_vehicle, closest_diff


def updateAccels(vehicles):
  new_accels = []
  for vehicle1 in vehicles:
    # enforce information constraints
    nearby_vehicles = findNearestOnPath(vehicle1, vehicles)
    (closest_vehicle,diff) = findClosestVehicleAhead(vehicle1, nearby_vehicles)
    if closest_vehicle:
      dist = 2 * math.pi * vehicle1.getRadius() * (diff / 360.0)
      lin_speed1 = abs(toLinear(vehicle1.getAngSpeed(), vehicle1.getRadius()))
      lin_speed2 = abs(toLinear(closest_vehicle.getAngSpeed(), closest_vehicle.getRadius()))
      new_accel = calculateAccel(dist, lin_speed1, lin_speed1 - lin_speed2)
      new_accels.append(new_accel)
    else:
      new_accels.append(MAX_ACCEL)

  for accel,vehicle in zip(new_accels,vehicles):
    vehicle.setAcceleration(accel)

# if __name__ == "__main__":
