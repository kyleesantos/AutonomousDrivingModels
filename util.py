import math

SCALE = 20 # pixels/m
UPDATE_TIME = 0.015

# Track Parameters
LEFT_ENTRANCE_THETA = 30
RIGHT_ENTRANCE_THETA = 150
LEFT_EXIT_THETA = 25
RIGHT_EXIT_THETA = 155
CTR_CLK = 1
CLK = -1
NEUTRAL = 0
COOP = 10
NON_COOP = 11

MAX_DEG = 360.0
MAX_RAD = 2 * math.pi

LEFT_WITHIN_INTERSECTION_THETA1 = 30
LEFT_WITHIN_INTERSECTION_THETA2 = 315
RIGHT_WITHIN_INTERSECTION_THETA1 = 150
RIGHT_WITHIN_INTERSECTION_THETA2 = 225

def toRadians(deg):
  return (deg / MAX_DEG) * MAX_RAD

def toDegrees(rad):
  return (rad / MAX_RAD) * MAX_DEG

def toAngular(linSpeed, radius):
  return (linSpeed * MAX_DEG) / (MAX_RAD * radius)

def toLinear(angSpeed, radius):
  return (angSpeed / MAX_DEG) * MAX_RAD * radius

def vehiclesCollide(veh1, veh2):
  ang1, ang2 = veh1.getTheta() % MAX_DEG, veh2.getTheta() % MAX_DEG
  diff = abs(ang1 - ang2)
  return (veh1.getDirection() == veh2.getDirection() and
    veh1.getRadius() == veh2.getRadius() and
    (veh1.getCarAngle() >= diff or (veh1.getCarAngle() >= abs(diff - 360))))

def vehicleIntersectionCollide(veh1, veh2):
  if (veh1.getDirection() == CLK):
    left = veh1
    if (veh2.getDirection() == CLK): return False
    right = veh2
  else:
    right = veh1
    if (veh2.getDirection() == CTR_CLK): return False
    left = veh2
  angL, angR = left.getTheta() % MAX_DEG, right.getTheta() % MAX_DEG
  diff = abs(180 - angL - angR)
  temp = (veh1.getRadius() == veh2.getRadius() and angR >= RIGHT_ENTRANCE_THETA and
    angR <= (MAX_DEG - RIGHT_ENTRANCE_THETA) and (angL <= LEFT_ENTRANCE_THETA or
    angL >= (MAX_DEG - LEFT_ENTRANCE_THETA)) and
    (veh1.getCarAngle() >= diff or (veh1.getCarAngle() >= abs(diff - 360))))
  return temp


  def euclidean(coord1, coord2):
    x1, y1 = coord1
    x2, y2 = coord2

    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


# return value is how far ahead (along circle) that car2 is from car1
# should only be called with two cars from the same side/circle else
# set theta to find distance between car1 and a location on its circle
def getArcDistance(car1, car2=None, theta=None):
  if (car1 == car2): return 0

  angle1 = car1.getTheta()
  if (theta is None):
    angle2 = car2.getTheta()
  else:
    angle2 = theta
  circumference = MAX_RAD * car1.getRadius()

  if (car1.getDirection() == CTR_CLK):
    if (angle2 < angle1): arcAngle = MAX_DEG - (angle1 - angle2)
    else: arcAngle = angle2 - angle1
  else:
    if (angle1 < angle2): arcAngle = MAX_DEG - (angle2 - angle1)
    else: arcAngle = angle1 - angle2
  return (arcAngle / MAX_DEG) * circumference

def withinIntersection(vehicle):
  if vehicle.getDirection() == CLK:
    return (LEFT_WITHIN_INTERSECTION_THETA1 >= vehicle.getTheta()) or (vehicle.getTheta() >= LEFT_WITHIN_INTERSECTION_THETA2)
  else:
    return (RIGHT_WITHIN_INTERSECTION_THETA1 <= vehicle.getTheta()) and (vehicle.getTheta() <= RIGHT_WITHIN_INTERSECTION_THETA2)
