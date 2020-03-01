from track import Track
from vehicle import Vehicle

LEFT = 0
RIGHT = 1

if __name__ == "__main__":
  vehiclesLeft = [Vehicle(1.0,1.0), Vehicle(2.0,4.0), Vehicle(5.0,3.0)]
  vehiclesRight = [Vehicle(5.0,1.0), Vehicle(6.0,4.0), Vehicle(9.0,3.0)]
  track = Track(vehiclesLeft, vehiclesRight)
  print(track.getIntersections())
  print(track.getCircle(LEFT))
  print(track.getCircle(RIGHT))