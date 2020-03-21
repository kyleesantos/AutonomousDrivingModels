from track import Track
from vehicle import Vehicle

vehiclesLeft = []
vehiclesRight = []

LEFT = 0
RIGHT = 1

def init_vehicles(side, points):
  if side == LEFT:
    vehiclesLeft = [Vehicle(point[0], point[1], point[2]) for point in points]
  elif side == RIGHT:
    vehiclesRight = [Vehicle(point[0], point[1], point[2]) for point in points]
  else:
    print("Error: Invalid side argument")


def init_track():
  track = Track(vehiclesLeft, vehiclesRight)
  for vehicle in vehiclesLeft:
    vehicle.set_path(track.getCircle(LEFT))

  for vehicle in vehiclesRight:
    vehicle.set_path(track.getCircle(RIGHT))


if __name__ == "__main__":
  track = Track(vehiclesLeft, vehiclesRight)
  print(track.getIntersections())
  print(track.getCircle(LEFT))
  print(track.getCircle(RIGHT))