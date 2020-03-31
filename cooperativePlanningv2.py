import math
import numpy as np

class Coop_Env():
	# Coop_Env houses stats/data on the current state of the cooperative environment

	def __init__(self, track_config="figure_8"):
		self.track_config = track_config
		self.intersection = None # should be set by setIntersection if track_config is figure_8
		self.intersectionThreshold = None # distance from the car to intersection with which a vehicle is considered to be approaching intersection
		self.nearIntersectionThreshold = 600
		self.pendingLeft = 0 # number of vehicles pending from left side
		self.pendingRight = 0 # number of vehicles pending from right side
		self.passingVehicle = None # vehicle currently passing through intersection
		self.weights = None
		self.leftEntranceTheta = 0.785398
		self.maxPassingCars = 3
		self.rightEntranceTheta = 2.35619
		self.vehiclesAtIntersection = None
		self.bufferDistance = 150 # buffer distance between any two cars (should be set appropriately)
		self.maxCarSeparation = self.bufferDistance + 5
		self.decision = None # decision is a list of 'decisions' where a decision is a tuple indicating a side and number of cars to let through
		self.lastPassingVehicle = None
		self.vehicles = None

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
	# should only be called with two cars from the same side/circle else
	# set theta to find distance between car1 and a location on its circle
	def getArcDistance(self, car1, car2=None, theta=None):
		if (car1 == car2): return 0

		angle1 = car1.getTheta()
		if (theta is None):
			angle2 = car2.getTheta()
		else:
			angle2 = theta
		circumference = 2 * math.pi * car1.getRadius()

		if (car1.getDirection() == 1):
			if (angle2 < angle1): arcAngle = (2*math.pi) - (angle1 - angle2)
			else: arcAngle = angle2 - angle1
		else:
			if (angle1 < angle2): arcAngle = (2*math.pi) - (angle2 - angle1)
			else: arcAngle = angle1 - angle2

		return (arcAngle/(2*math.pi)) * circumference

	def getVehiclesAtIntersection(self, vehicles):
		result = []
		for vehicle in vehicles:
			if (vehicle == self.passingVehicle): continue
			if self.nearIntersection(vehicle): result.append(vehicle)

		result.sort(key=lambda vehicle: self.getDistanceToIntersection(vehicle))

		return result[:2]

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
		left = [vehicle for vehicle in vehicles if (vehicle.getDirection() == 0)]
		right = [vehicle for vehicle in vehicles if (vehicle.getDirection() == 1)]

		
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
		if (vehicle.getDirection() == 0): self.pendingLeft += dx
		else: self.pendingRight += dx

	# updates the positions of the ordered list of vehicles
	def updateVehiclePositions(self, orderedVehicles):
		for i in range(len(orderedVehicles)):
			vehicle = orderedVehicles[i]

			if (i == 0): precedingVehicle = orderedVehicles[len(orderedVehicles)-1]
			else: precedingVehicle = orderedVehicles[i-1]
			
			if (self.nearIntersection(vehicle) and (not vehicle.isPassingIntersection())): continue

			arcDistance = self.getArcDistance(vehicle, precedingVehicle)
			if ((arcDistance >= self.bufferDistance) or (len(orderedVehicles) == 1)):

				vehicle.turn()

				# check if vehicle just passed the intersection
				if (vehicle.isPassingIntersection()):
					posY = vehicle.getPos()[1]
					if (posY > self.intersection[1]):
						vehicle.setPassingIntersection(passing=False)
						if (self.lastPassingVehicle == vehicle):
							self.lastPassingVehicle = None
							self.decision[0] = self.decision[1]
							self.decision[1] = None
							if (self.decision[0] is None): self.decision = None
							else: self.distributeDecision()

	# returns a feature vector from the given vehicle to use in 
	# determining the vehicle to pass through intersection
	def getFeatureVector(self, vehicle):
		if (vehicle.getDirection() == 0): pendingCount = self.pendingLeft
		else: pendingCount = self.pendingRight 
		return np.array([1/self.getDistanceToIntersection(vehicle), pendingCount])

	# returns the length of the longest continuous string of cars that are 
	# each within a certain distance from each other, starting from the first car
	def getNumCarsInString(self, distances):
		if len(distances) == 0: return 0
		count = 1
		for i in range(1, len(distances)):
			separation = distances[i] - distances[i - 1]
			if separation > self.maxCarSeparation: break
			else:
				count += 1
		return count

	def getSideScore(self, tup):
		numCars, distances = tup

		if len(distances) == 0: return -1

		minDist = distances[0]
		vector = np.array([minDist, numCars])

		return np.dot(self.weights, vector)

	# returns the vehicles approaching the intersection from both lanes, along with their distances to intersection
	# returns the vehicles sorted by distance to intersection
	def getVehiclesApproachingIntersection(self):
		leftVehicles = []
		rightVehicles = []

		for vehicle in self.vehicles:
			if vehicle.getDirection() == 0:
				arcDistance = self.getArcDistance(car1=vehicle, theta=self.leftEntranceTheta)
				if (arcDistance < self.nearIntersectionThreshold):
					leftVehicles.append((vehicle, arcDistance))
			else:
				arcDistance = self.getArcDistance(car1=vehicle, theta=self.rightEntranceTheta)
				if (arcDistance < self.nearIntersectionThreshold):
					rightVehicles.append((vehicle, arcDistance))

		leftVehicles.sort(key=lambda tup: tup[1])
		rightVehicles.sort(key=lambda tup: tup[1])

		return leftVehicles, rightVehicles


	def getNextDecision(self):
		leftDecision = None
		rightDecision = None

		# get number of vehicles close to intersection at both lanes
		leftVehicles, rightVehicles = self.getVehiclesApproachingIntersection()
		leftDistances = [tup[1] for tup in leftVehicles]
		rightDistances = [tup[1] for tup in rightVehicles]

		numLeft = self.getNumCarsInString(leftDistances)
		numRight = self.getNumCarsInString(rightDistances)

		leftScore = self.getSideScore((numLeft, leftDistances))
		rightScore = self.getSideScore((numRight, rightDistances))

		# no result case
		if ((leftScore == -1) and (rightScore == -1)): return None

		if (leftScore >= 0):
			leftDecision = (0, min(numLeft, self.maxPassingCars))
		if (rightScore >= 0):
			rightDecision = (1, min(numRight, self.maxPassingCars))

		if (leftScore > rightScore):
			return [leftDecision, rightDecision]
		else:
			return [rightDecision, leftDecision]

	def distributeDecision(self):
		leftVehicles, rightVehicles = self.getVehiclesApproachingIntersection()

		side = self.decision[0][0]
		numCars = self.decision[0][1]

		if side == 0: passingVehicles = leftVehicles
		else: passingVehicles = rightVehicles 

		for i in range(numCars):
			vehicle = passingVehicles[i][0]
			vehicle.setPassingIntersection(passing=True)
			if (i == numCars-1):
				self.lastPassingVehicle = vehicle


	# returns vehicle objects each updated with its next position
	def step(self, vehicles):
		self.vehicles = vehicles

		if (self.decision is None):
			self.decision = self.getNextDecision()

			if (not self.decision is None):
				print("decision is ", self.decision)
				self.distributeDecision()

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