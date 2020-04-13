from util import *

class Warning():
	# encapsulates information of a warning signal

	def __init(self, code, delay, location=None, sender):
		self.code = code
		self.delay = delay
		self.location = location
		self.sender = sender

	# a warning is the same as another if they reference the same problem, regardless of the sender
	def __eq__(self, other):
		if (isinstance(other, Warning)):
			if (self.code == other.code):
				return self.location == other.location
			else:
				return False
		else:
			return False

class Planning():
	# planning class controls the v2v interaction and cooperative path planning between vehicles

	def __init__(self, mode=NON_COOP):
		self.mode = mode
		self.vehicles = None
		self.warningDelayRange = (4, 7)
		self.warningsDict = dict()

	# appends warning to vehicles list of warnings pending
	def _sendWarning(self, warning, vehicle):
		if (not vehicle.getID() in self.warningsDict):
			self.warningsDict[vehicle.getID()] = []

		self.warningsDict[vehicle.getID()].append(warning)

	# communicates warning to other vehicles within vehicle's communication range
	def _communicateWarning(self, vehicle1, warning):
		for vehicle2 in self.vehicles:
			if (vehicle2.getID() == vehicle1.getID()): continue
			distance = euclidean(vehicle1.getPos(), vehicle2.getPos())
			if (distance <= vehicle1.getCommunicationRange()):
				# random delay to simulate communication latency
				delay = random.randint(self.warningDelayRange[0], self.warningDelayRange[1])

				newWarning = Warning(warning.code, delay, warning.location, vehicle1)
				self._sendWarning(newWarning, vehicle2) 

	# keeps track of warnings that have been sent v2v and are still pending
	# if warning is no longer pending, it delivers the warning to the respective vehicle
	def _deliverWarnings(self):
		for vehicleId in self.warningsDict:
			warnings = self.warningsDict[vehicleId]
			pendingWarnings = []
			readyWarnings = []
			for warning in warnings:
				warning.delay -= 1
				if (warning.delay <= 0):
					readyWarnings.append(warning)
				else:
					pendingWarnings.append(warning)
			self.warningsDict[vehicleId] = pendingWarnings
			self._deliverWarningsToVehicle(readyWarnings, vehicleId)

	def _deliverWarningsToVehicle(self, warnings, vehicleId):
		for v in self.vehicles:
			if (v.getID() == vehicleId):
				# send the warnings to vehicle and communicate novel warnings to neighbouring vehicles
				unseenWarnings = v.addWarnings(warnings)

				for warning in unseenWarnings:
					self._communicateWarning(v, warning)
				return


	# ensures that all vehicles are processing any warnings that they might have
	def _processWarnings(self):
		for vehicle in self.vehicles:
			if ((len(vehicle.getWarnings()) > 0) and not vehicle.isHandlingWarning()):
				vehicle.handleWarning()

	def step(self, vehicles):
		self.vehicles = vehicles

		#self._detectProblem()
		self._deliverWarnings()
		self._processWarnings()