import math
import numpy as np
import random
from util import *
from vehicle import VEH_LENGTH, WHE_LENGTH
import idm

# vehicles with less than this arc distance to intersection are considered as 'approaching' intersection
REAL_NEAR_INTER_THRESH = 24

# vehicles with less than this arc distance to intersection are considerted to be 'at' intersection
REAL_AT_INTER_THRESH = 6
REAL_MAX_CAR_SEPARATION = 10
REAL_CRITICAL_THRESH = 6
MAX_PASSING_CARS = 3

NEAR_INTER_THRESH = SCALE * REAL_NEAR_INTER_THRESH
AT_INTER_THRESH = SCALE * REAL_AT_INTER_THRESH
MAX_CAR_SEPARATION = SCALE * REAL_MAX_CAR_SEPARATION
CRITICAL_THRESH = SCALE * REAL_CRITICAL_THRESH

class Env():
	# Env houses stats/data on the current state of the environment. initialize NON_COOP OR COOP mode depending on desired control mechanism

	def __init__(self, track_config="figure_8", mode=NON_COOP):
		self.track_config = track_config
		self.mode = mode
		self.intersection = None # should be set by setIntersection if track_config is figure_8
		self.nearIntersectionThreshold = NEAR_INTER_THRESH
		self.atIntersectionThreshold = AT_INTER_THRESH
		self.weights = None
		self.maxPassingCars = MAX_PASSING_CARS
		self.vehiclesAtIntersection = None
		self.maxCarSeparation = MAX_CAR_SEPARATION
		self.decision = None # decision is a list o 'decisions' where a decision is a tuple indicating a side and number of cars to let through
		self.lastPassingVehicle = None
		self.vehicles = None
		self.vehiclesApproachingIntersection = None
		self.dataDict = dict()
		self.dataDelayRange = (10, 13)


	def isApproachingIntersection(self, vehicle):
		if vehicle.getDirection() == CLK:
			arcDistance = getArcDistance(vehicle, theta=LEFT_ENTRANCE_THETA)
		else:
			arcDistance = getArcDistance(vehicle, theta=RIGHT_ENTRANCE_THETA)

		return (arcDistance < self.nearIntersectionThreshold)

	# when a vehicle approaches the intersection, this function will simulate
	# v2v communication and tell the preceding vehicle that there is a car behind it
	def communicatePresence(self):
		for vehicle in self.vehicles:
			if (self.isApproachingIntersection(vehicle) and not vehicle.hasCommunicatedPresence()):
				vehicle.setNumCarsBehind(0)
				self.communicateData(vehicle, 1)
				vehicle.setCommunicatedPresence(True)

	# adds data to data dict for the specified vehicle
	def sendData(self, numCars, vehicle, senderID):
		if (not vehicle.getID() in self.dataDict):
			self.dataDict[vehicle.getID()] = []

		delay = random.randint(self.dataDelayRange[0], self.dataDelayRange[1])
		data = [numCars, delay, senderID]
		self.dataDict[vehicle.getID()].append(data)

	def deliverData(self):
		for vehicleId in list(self.dataDict):
			datas = self.dataDict[vehicleId]
			readyDatas = [data for data in datas if data[1] == 1]
			assert(len(readyDatas) <= 1)
			if (len(datas) == 0): continue
			data = datas[0]
			data[1] -= 1
			if (data[1]) <= 0:
				self.deliverDataToVehicle(data, vehicleId)
				self.dataDict[vehicleId] = []

	def deliverDataToVehicle(self, data, vehicleId):
		for vehicle in self.vehicles:
			if (vehicle.getID() != vehicleId): continue
			numCars = data[0]
			if (self.mode == COOP):
				vehicle.setNumCarsBehind(numCars)
			self.communicateData(vehicle, numCars + 1)


	# communicates numCars to the preceding vehicle of vehicle(if one exists)
	def communicateData(self, vehicle, numCars):
		leftVehicles, rightVehicles = self.getVehiclesApproachingIntersection()

		if (vehicle.getDirection() == CLK):
			laneVehicles = leftVehicles
		else:
			laneVehicles = rightVehicles

		for i in range(len(laneVehicles)):
			v = laneVehicles[i][0]
			if (v.getID() == vehicle.getID()):
				if (i == 0): return
				precedingVehicle = laneVehicles[i-1][0]
				self.sendData(numCars, precedingVehicle, vehicle.getID())
				return

	# clears necessary communication data of the vehicle
	def clearData(self, vehicle):
		vehicle.setCommunicatedPresence(False)
		vehicle.setNumCarsBehind(0)

		self.dataDict[vehicle.getID()] = []


	def getNextVehicleInfo(self, vehicle):
		closestDiff = MAX_DEG
		closestVehicle = None
		for vehicle2 in self.vehicles:
			if vehicle.getDirection() == vehicle2.getDirection():
				dist = toAngular(getArcDistance(vehicle, car2=vehicle2), vehicle.getRadius())
				if dist < closestDiff and dist > 0:
					closestDiff = dist
					closestVehicle = vehicle2
		return closestVehicle, closestDiff


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


	def getChainsOfCars(self, vehiclesDistances, following=False):
		if len(vehiclesDistances) == 0:
			return []

		allChains = []
		currChain = [vehiclesDistances[0][0]]
		for i in range(1,len(vehiclesDistances)):
			separation = vehiclesDistances[i][1] - vehiclesDistances[i - 1][1]
			if separation > self.maxCarSeparation:
				allChains.append(currChain)
				currChain = [vehiclesDistances[i][0]]
			else:
				currChain.append(vehiclesDistances[i][0])
		if (following and len(allChains) > 0 and
			allChains[0][0].getDirection() == self.getRightOfWay()):
			lastDist = getArcDistance(allChains[0][0], car2=currChain[-1])
			if lastDist < self.maxCarSeparation:
				allChains[0] = currChain + allChains[0]
			else:
				allChains.append(currChain)
		else:
			allChains.append(currChain)
		return allChains


	# assings a score to the left and right sides of the track, indicating priority when choosing
	# which lane to 'open' at the intersection
	def getSideScore(self, tup):
		vehicleAtIntersection, numCars, distances = tup

		if ((len(distances) == 0) or (not vehicleAtIntersection)): return -1

		minDist = distances[0]

		# div by zero prevention
		if (minDist == 0):
			minDist = 0.25

		vector = np.array([1/minDist, numCars])

		if (self.mode == NON_COOP):
			weights = np.array([1, 0])
		elif (self.mode == COOP):
			weights = self.weights

		return np.dot(weights, vector)

	# returns the vehicles approaching the intersection from both lanes, along with their distances to intersection
	# returns the vehicles sorted by distance to intersection
	# returns vehicles and their distance to the intersection
	def getVehiclesApproachingIntersection(self):
		leftVehicles = []
		rightVehicles = []

		for vehicle in self.vehicles:
			if vehicle.getDirection() == CLK:
				arcDistance = getArcDistance(vehicle, theta=LEFT_ENTRANCE_THETA)
				if (arcDistance < self.nearIntersectionThreshold):
					leftVehicles.append((vehicle, arcDistance))
			else:
				arcDistance = getArcDistance(vehicle, theta=RIGHT_ENTRANCE_THETA)
				if (arcDistance < self.nearIntersectionThreshold):
					rightVehicles.append((vehicle, arcDistance))

		leftVehicles.sort(key=lambda tup: tup[1])
		rightVehicles.sort(key=lambda tup: tup[1])

		return leftVehicles, rightVehicles

	def getDistancesPerLane(self):
		leftVehicles = []
		rightVehicles = []

		for vehicle in self.vehicles:
			if vehicle.getDirection() == CLK:
				arcDistance = getArcDistance(vehicle, theta=LEFT_ENTRANCE_THETA)
				leftVehicles.append((vehicle, arcDistance))
			else:
				arcDistance = getArcDistance(vehicle, theta=RIGHT_ENTRANCE_THETA)
				rightVehicles.append((vehicle, arcDistance))

		leftVehicles.sort(key=lambda tup: tup[1])
		rightVehicles.sort(key=lambda tup: tup[1])

		return leftVehicles, rightVehicles

	def checkVehiclesAtIntersection(self):
		atLeft, atRight = False, False

		for vehicle in self.vehicles:
			if (vehicle.getDirection() == CLK):
				arcDistance = getArcDistance(car1=vehicle, theta=LEFT_ENTRANCE_THETA)
				if (arcDistance < self.atIntersectionThreshold):
					atLeft = True
			else:
				arcDistance = getArcDistance(car1=vehicle, theta=RIGHT_ENTRANCE_THETA)
				if (arcDistance < self.atIntersectionThreshold):
					atRight = True

		return atLeft, atRight

	# makes a decision on which cars to let through the intersection
	# returns none if no cars are at the intersection
	def getNextDecision(self):
		leftDecision = None
		rightDecision = None

		# check if any vehicles are at the intersection
		atLeft, atRight = self.checkVehiclesAtIntersection()

		# get number of vehicles close to intersection at both lanes
		leftVehicles, rightVehicles = self.getVehiclesApproachingIntersection()

		leftDistances = [tup[1] for tup in leftVehicles]
		rightDistances = [tup[1] for tup in rightVehicles]


		# one person goes in non coop mode
		if self.mode == NON_COOP:
			numLeft = 1
			numRight = 1
		elif self.mode == COOP:
			numLeft = self.getNumCarsInString(leftDistances)
			numRight = self.getNumCarsInString(rightDistances)


		leftScore = self.getSideScore((atLeft, numLeft, leftDistances))
		rightScore = self.getSideScore((atRight, numRight, rightDistances))

		# no result case
		if ((leftScore == -1) and (rightScore == -1)): return None

		self.vehiclesApproachingIntersection = leftVehicles, rightVehicles

		if (leftScore >= 0):
			if rightScore < 0:
				leftDecision = (CLK, numLeft)
			else:
				leftDecision = (CLK, min(numLeft, self.maxPassingCars))
		if (rightScore >= 0):
			if leftScore < 0:
				rightDecision = (CTR_CLK, numRight)
			else:
				rightDecision = (CTR_CLK, min(numRight, self.maxPassingCars))

		if self.mode == NON_COOP:
			if (leftScore >= rightScore):
				return [leftDecision, None]
			else:
				return [rightDecision, None]
		elif self.mode == COOP:
			if (leftScore > rightScore):
				return [leftDecision, rightDecision]
			else:
				return [rightDecision, leftDecision]

	def setVehiclesInCriticalSection(self):
		# get vehicles close to intersection at both lanes
		for vehicle in self.vehicles:
			if (vehicle.getDirection() == CLK):
				arcDistance = getArcDistance(car1=vehicle, theta=LEFT_ENTRANCE_THETA)
				if (arcDistance < CRITICAL_THRESH):
					vehicle.setInCriticalSection(True)
			elif (vehicle.getDirection() == CTR_CLK):
				arcDistance = getArcDistance(car1=vehicle, theta=RIGHT_ENTRANCE_THETA)
				if (arcDistance < CRITICAL_THRESH):
					vehicle.setInCriticalSection(True)

	def distributeDecision(self):
		#leftVehicles, rightVehicles = self.getVehiclesApproachingIntersection()
		leftVehicles, rightVehicles = self.vehiclesApproachingIntersection
		side = self.decision[0][0]
		numCars = self.decision[0][1]

		if side == CLK: passingVehicles = leftVehicles
		else: passingVehicles = rightVehicles

		for i in range(numCars):
			vehicle = passingVehicles[i][0]
			vehicle.setPassingIntersection(passing=True)
			if (i == numCars-1):
				self.lastPassingVehicle = vehicle

	# updates environment state and makes decision on vehicles to pass through the intersection
	def step(self, vehicles):
		self.vehicles = vehicles

		self.communicatePresence()
		self.deliverData()

		if (not self.decision):
			self.decision = self.getNextDecision()

			if (self.decision):
				self.distributeDecision()

		self.setVehiclesInCriticalSection()

		for vehicle in vehicles:
			if (vehicle.isPassingIntersection()):
				posY = vehicle.getPos()[1]
				if (vehicle.getDirection() == CLK):
					distToIntersection = getArcDistance(car1=vehicle, theta=LEFT_ENTRANCE_THETA)
				else:
					distToIntersection = getArcDistance(car1=vehicle, theta=RIGHT_ENTRANCE_THETA)
				if ((posY > self.intersection[1]) and (distToIntersection > self.nearIntersectionThreshold)):
					vehicle.setPassingIntersection(passing=False)
					vehicle.setInCriticalSection(False)
					self.clearData(vehicle)
					if (self.lastPassingVehicle == vehicle):
						self.lastPassingVehicle = None
						self.decision[0] = self.decision[1]
						self.decision[1] = None
						if (self.decision[0] is None): self.decision = None
						else:
							self.distributeDecision()

		idm.updateAccels(vehicles)


		if self.mode == COOP:
			leftDistances, rightDistances = self.getDistancesPerLane()
			chains = (self.getChainsOfCars(leftDistances, following=True) +
						self.getChainsOfCars(rightDistances, following=True))
			for chain in chains:
				leadingAccel = chain[0].getAcceleration()
				for (i,vehicle) in enumerate(chain):
					tempAccel = vehicle.getAcceleration()
					if vehicle.isPassingIntersection() or not vehicle.isInCriticalSection():
						newAccel = max(leadingAccel, tempAccel)

						vehicle2, dist = self.getNextVehicleInfo(vehicle)
						updatedSpeed2 = abs(vehicle2.getAngSpeed()) + vehicle2.getAcceleration()
						maxAccel = (updatedSpeed2 - abs(vehicle.getAngSpeed()) +
							(dist - toAngular(2*VEH_LENGTH + 4*WHE_LENGTH, vehicle.getRadius())))

						vehicle.setAcceleration(min(newAccel, maxAccel), angular=True)
					else:
						for j in range(i+1,len(chain)):
							currAccel = chain[j].getAcceleration()
							newAccel = max(currAccel, tempAccel)

							vehicle2, dist = self.getNextVehicleInfo(chain[j])
							updatedSpeed2 = abs(vehicle2.getAngSpeed()) + vehicle2.getAcceleration()
							maxAccel = (updatedSpeed2 - abs(chain[j].getAngSpeed()) +
								(dist - toAngular(2*VEH_LENGTH + 4*WHE_LENGTH, chain[j].getRadius())))

							chain[j].setAcceleration(min(newAccel, maxAccel), angular=True)
						break


	def getRightOfWay(self):
		if self.decision:
			return self.decision[0][0]

		return NEUTRAL

	# use this to set the center of the intersection point if track_config is figure_8
	def setIntersection(self, pos):
		self.intersection = pos

	def setWeights(self, weights):
		self.weights = weights
