import math
from vehicle import Vehicle

SCALE = 25 # pixels/m

# Intelligent Driver Model Parameters
OPT_VELOCITY = 0.4  # m/s
TIME_HEADWAY = 2    # s
MAX_ACCEL = 0.5     # m/s2
OPT_DECEL = 0.3     # m/s2
BUFFER_DIST = 0.1   # m
ACCEL_EXP = 4

# Information Constraints Parameters
DETECTION_DIST = 3  # m

def calculate_accel(dist, v, delta_v):
  a = math.pow((v/delta_v), ACCEL_EXP)
  min_gap = BUFFER_DIST + TIME_HEADWAY*v + (v*delta_v/(2*math.sqrt(MAX_ACCEL*OPT_DECEL)))
  b = math.pow((min_gap / dist),2)
  accel = MAX_ACCEL*(1 - a - b)
  return accel


# returns true if vehicle2 is within the information constraint
def is_detectable(vehicle1, vehicle2):
  return ((vehicle1 != vehicle2) and
          (math.pow(vehicle1.x - vehicle2.x, 2) +
            math.pow(vehicle1.y - vehicle2.y, 2) <= math.pow(DETECTION_DIST, 2)))

def is_on_path(vehicle1, vehicle2):
  return ((vehicle1.trackX == vehicle2.trackX) and
          (vehicle1.trackY == vehicle2.trackY))


def find_nearest_on_path(vehicle, vehicles):
  nearby_vehicles = []
  for vehicle2 in vehicles:
    if is_detectable(vehicle, vehicle2) and is_on_path(vehicle, vehicle2):
      nearby_vehicles.append(vehicle2)
  return nearby_vehicles


def find_closest_vehicle_ahead(vehicle, vehicles):
  closest_diff = 2*math.pi()
  closest_vehicle = None
  for vehicle2 in vehicles:
    diff = vehicle2.theta - vehicle.theta if vehicle2.theta > vehicle.theta else vehicle2.theta + 2*math.pi() - vehicle.theta
    if diff < closest_diff:
      closest_diff = diff
      closest_vehicle = vehicle2
  return closest_vehicle, closest_diff


def update_accels(vehicles):
  new_accels = []
  for vehicle1 in vehicles:
    # enforce information constraints
    nearby_vehicles = find_nearest_on_path(vehicle1, vehicles)
    (closest_vehicle,diff) = find_closest_vehicle_ahead(vehicle1, nearby_vehicles)
    dist = vehicle1.r*diff
    lin_speed1 = vehicle1.speed*vehicle1.r
    lin_speed2 = closest_vehicle.speed*closest_vehicle.r
    new_accel = calculate_accel(dist, lin_speed1, lin_speed1-lin_speed2)
    new_accels.append(new_accel)

  # for accel,vehicle in zip(new_accels,vehicles):
  #   vehicle.accel = accel

# if __name__ == "__main__":
