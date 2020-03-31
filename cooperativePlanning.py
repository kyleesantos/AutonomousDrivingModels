import math
import numpy as np
from vehicle import LEFT, RIGHT

class Coop_Env():
	# Coop_Env houses stats/data on the current state of the cooperative environment

	def __init__(self, track_config="figure_8"):
		self.track_config = track_config
		self.intersection = None # should be set by setIntersection if track_config is figure_8
		self.intersectionThreshold = None # distance from the car to intersection with which a vehicle is considered to be approaching intersection
		self.pendingLeft = 0 # number of vehicles pending from left side
		self.pendingRight = 0 # number of vehicles pending from right side
		self.passingVehicle = None # vehicle currently passing through intersection
		self.numLeft = 0 # number of cars on the left side
		self.numRight = 0 # number of cars on the right side
		self.weights = None
		self.bufferDistance = 150 # buffer distance between any two cars (should be set appropriately)

	# returns true if the vehicle is approaching the intersection
	def nearIntersection(self, vehicle):
		_, intersectionY = self.intersection
		posY = vehicle.getPos()[1]

		# if car is below intersection point, then it has already passed the intersection
		if (posY > intersectionY): return False

		distance = self.getDistanceToIntersection(vehicle)

		if (distance <= self.intersectionThreshold): return True

		return False

	# return value is how far ahead (along circle) that car2 is from car1
	# should only be called with two cars from the same side/circle
	def getArcDistance(self, car1, car2):
		if (car1 == car2): return 0

		angle1 = car1.getTheta()
		angle2 = car2.getTheta()
		circumference = 360 * car1.getRadius()

		assert (car1.getDirection() == car2.getDirection()), "calling arc distance with cars from different sides"
		if (car1.getDirection() == 1):
			if (angle2 < angle1): arcAngle = (360 + angle2) - angle1
			else: arcAngle = angle2 - angle1
		else:
			if (angle1 < angle2): arcAngle = angle2 + angle1
			else: arcAngle = angle1 - angle2

		return (arcAngle/(360)) * circumference

	def getVehiclesAtIntersection(self, vehicles):
		result = []
		for vehicle in vehicles:
			if (vehicle == self.passingVehicle): continue
			if self.nearIntersection(vehicle): result.append(vehicle)
		return result

	# chooses a vehicle to pass through the intersection based on
	# weighted features of the environment state
	def selectVehicle(self, vehiclesAtIntersection):
		if len(vehiclesAtIntersection) == 0: return None

		# sort vehicles based on priority
		sortedVehicles = sorted(vehiclesAtIntersection, key=lambda vehicle: np.dot(self.weights, self.getFeatureVector(vehicle)), reverse=True)
		return sortedVehicles[0]

	# returns lists of ordered vehicles, one for each side.
	# an order of vehicles starts from one vehicle (in one of the circles/loops) and continues backwards
	def getOrderedVehiclesPerSide(self, vehicles):
		left = [vehicle for vehicle in vehicles if (vehicle.getDirection() == LEFT)]
		right = [vehicle for vehicle in vehicles if (vehicle.getDirection() == RIGHT)]


		if (len(left) > 0):
			leftReference = left[0] # reference vehicle to determine order. reference vehicle will be first vehicle of the order
			left.sort(key=lambda vehicle: self.getArcDistance(vehicle, leftReference))

		if (len(right) > 0):
			rightReference = right[0] # reference vehicle to determine order. reference vehicle will be first vehicle of the order
			right.sort(key=lambda vehicle: self.getArcDistance(vehicle, rightReference))


		return left, right

	# gets the euclidean distance of a vehicle to the intersection
	def getDistanceToIntersection(self, vehicle):
		posX, posY = vehicle.getPos()
		intersectionX, intersectionY = self.intersection

		return math.sqrt((intersectionX - posX)**2 + (intersectionY - posY)**2)

	def updatePendingCount(self, vehicle, dx):
		if (vehicle.getDirection() == LEFT): self.pendingLeft += dx
		else: self.pendingRight += dx

	# updates the positions of the ordered list of vehicles
	def updateVehiclePositions(self, orderedVehicles):
		for i in range(len(orderedVehicles)):
			vehicle = orderedVehicles[i]

			if (i == 0): precedingVehicle = orderedVehicles[len(orderedVehicles)-1]
			else: precedingVehicle = orderedVehicles[i-1]

			if (self.nearIntersection(vehicle) and (self.passingVehicle != vehicle)): continue

			arcDistance = self.getArcDistance(vehicle, precedingVehicle)
			if ((arcDistance >= self.bufferDistance) or (len(orderedVehicles) == 1)):
				vehicle.update()

				# check if vehicle just passed the intersection
				if (vehicle == self.passingVehicle):
					posY = vehicle.getPos()[1]
					if (posY > self.intersection[1]):
						self.passingVehicle = None

				if (vehicle.isPending()):
					vehicle.setPending(pending=False)
					self.updatePendingCount(vehicle, -1)
			else:
				if (not vehicle.isPending()):
					vehicle.setPending(pending=True)
					self.updatePendingCount(vehicle, 1)

	# returns a feature vector from the given vehicle to use in
	# determining the vehicle to pass through intersection
	def getFeatureVector(self, vehicle):
		if (vehicle.getDirection() == LEFT): pendingCount = self.pendingLeft
		else: pendingCount = self.pendingRight
		return np.array([1/self.getDistanceToIntersection(vehicle), pendingCount])

	# returns vehicle objects each updated with its next position
	def step(self, vehicles):
		vehiclesAtIntersection = self.getVehiclesAtIntersection(vehicles)


		# set vehicles at intersection to pending state/update pending counters
		for vehicle in vehiclesAtIntersection:
			if (vehicle.isPending() or (self.passingVehicle == vehicle)): continue
			vehicle.setPending()
			if (vehicle.getDirection() == LEFT): self.pendingLeft += 1
			else: self.pendingRight += 1

		# if no vehicle is passing through intersection, choose next vehicle to pass through
		if (self.passingVehicle is None):

			selectedVehicle = self.selectVehicle(vehiclesAtIntersection)

			# vehicle is no longer pending and is now going to pass through intersection
			if (not selectedVehicle is None):
				selectedVehicle.setPending(pending=False)
				if (selectedVehicle.getDirection() == LEFT): self.pendingLeft -= 1
				else: self.pendingRight -= 1
				self.passingVehicle = selectedVehicle

		# get ordered list of vehicles on either side
		orderedLeft, orderedRight = self.getOrderedVehiclesPerSide(vehicles)

		self.updateVehiclePositions(orderedLeft)
		self.updateVehiclePositions(orderedRight)


	# use this to set the center of the intersection point if track_config is figure_8
	def setIntersection(self, pos):
		self.intersection = pos

	def setWeights(self, weights):
		self.weights = weights

	def setIntersectionThreshold(self, value):
		self.intersectionThreshold = value
