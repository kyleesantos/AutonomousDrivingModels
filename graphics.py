
import tkinter as tk
import itertools, math, time
import numpy as np

from pathPlanning import Env
from tkinter import *
from util import *
import vehicle
import idm


root = tk.Tk()

CANVAS_WIDTH = 1200
CANVAS_HEIGHT = 800
TRACK_WIDTH = vehicle.VEH_WIDTH * 4 # 120
MARGIN = 100
TWO_LANE = False

#size of track
outR = (CANVAS_WIDTH + TRACK_WIDTH - 2 * MARGIN) // 4 # 280
vehR = outR - (TRACK_WIDTH // 2) # 220
# left center track (380, 400)
trackLeftX = MARGIN + outR
trackRightX = CANVAS_WIDTH - MARGIN - outR
# right center track (820, 400)
trackY = CANVAS_HEIGHT // 2
# center track
trackX = CANVAS_WIDTH // 2

canvas = tk.Canvas(root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT)
canvas.pack()

timerCounter = None
timerLabel = Label(text = "0.0 s", fg = "red")
infoLabel = Label(text = "")
loopLabel = Label(text = "0", fg = "darkBlue")
modeLabel = Label(text = "", fg = "DarkOrchid4")
trackBCoords, detBCoords, modeBCoords = [], [], []

move = False
testing, testingTime = False, 5.0
testList = [(0, CLK), (0, CTR_CLK)] # degrees, direction
vehicles = []
detectionRadius = False
totalLoops = 0
mode = COOP

def initEnv():
	global env
	env = Env(mode=mode)
	env.setIntersection((610, 367))
	#env.setIntersection((trackX, trackY))
	env.setWeights(np.array([1,1]))

def testCase(m):
	global timerCounter, move, testing, mode, testList
	mode = m
	reset()
	for (theta, direc) in testList:
		makeVehicle(theta, direc, vehR)
	timerCounter = time.time()
	move, testing = True, True

def keyPress(event):
	global move, timerCounter
	if (event.char == "s"):
		if timerCounter == None: timerCounter = time.time()
		move = not move
	if (event.char == "r"):
		timerCounter = time.time()
		timerLabel.config(text = "0.0 s")
		reset()
	if (event.char == "t"):
		testCase(NON_COOP)
	if (event.char == "1"):
		for v in vehicles:
			v.increaseAngSpeed(1)
	if (event.char == "2"):
		for v in vehicles:
			v.decreaseAngSpeed(1)

def mousePress(event):
	x, y = event.x, event.y
	changeTrack(x, y)
	addDetectionRadius(x, y)
	changeMode(x, y)
	r, direc = inTrack(x, y)
	if (r != None):
		theta = placeOnTrack(x, y, direc, r)
		makeVehicle(theta, direc, r)

def modeName():
	global mode
	if (mode == COOP): return "Cooperative"
	return "Non-Cooperative"

def inTrack(x, y):
	global vehR
	# Check for overlap
	direc, tX, tY, tR = CLK, trackLeftX, trackY, vehR
	if (TWO_LANE): tX = trackX
	elif (x > CANVAS_WIDTH // 2): direc, tX = CTR_CLK, trackRightX
	for v in vehicles:
		x1 = x - tX
		y1 = y - tY
		r = math.sqrt((x1)**2 + (y1)**2)
		a = toDegrees(math.acos(float(x1 / r)))
		if (y1 > 0): a = (a * -1.0) % MAX_DEG
		if (TWO_LANE and r > outR): tR += TRACK_WIDTH
		checkV = vehicle.Vehicle(tX, tY, tR, a, direc, -1)
		if (vehiclesCollide(v, checkV)): return (None, None)

	# Check for 2 lane track first
	r = math.sqrt((tX - x)**2 + (tY - y)**2)
	if (TWO_LANE and ((r <= outR and r >= (outR - TRACK_WIDTH)) or
			(r <= outR + TRACK_WIDTH and r >= outR))): return (r, CLK)
	elif (r <= outR and r >= (outR - TRACK_WIDTH)): return (r, direc)
	return (None, None)

def placeOnTrack(x, y, direc, r):
	if (TWO_LANE): x -= trackX
	elif (direc == CLK): x -= trackLeftX
	else: x -= trackRightX
	y -= trackY
	a = toDegrees(math.acos(float(x / r)))
	if (y > 0): a = (a * -1.0)
	return a

def flatten(l):
	return [item for tup in l for item in tup]

def makeVehicle(theta, direc, r):
	global info, vehR
	# Add Vehicle
	radius = vehR
	if (TWO_LANE):
		tX = trackX
		if (r <= outR + TRACK_WIDTH and r >= outR): radius += TRACK_WIDTH
	elif (direc == CTR_CLK): tX = trackRightX
	else: tX = trackLeftX

	v = vehicle.Vehicle(tX, trackY, radius, theta, direc, len(vehicles))
	vehCanvas = canvas.create_polygon(v.getVehPoints(), fill='red')
	wheelsCanvas = []
	for wP in v.getWheelPoints():
		wheelsCanvas.append(canvas.create_polygon(wP, fill='black'))
	dirCanvas = canvas.create_polygon(v.getDirPoints(), fill = 'yellow')
	idCanvas = canvas.create_text(v.getX(), v.getY(), text = str(v.getID()))
	detCanvas = canvas.create_oval(v.getX() - idm.DETECTION_DIST, 
		v.getY() - idm.DETECTION_DIST, v.getX() + idm.DETECTION_DIST, 
		v.getY() + idm.DETECTION_DIST, outline = "DarkOrange2")
	if (detectionRadius): canvas.itemconfig(detCanvas, state = tk.NORMAL)
	else: canvas.itemconfig(detCanvas, state = tk.HIDDEN)
	v.setCanvas([vehCanvas, wheelsCanvas, dirCanvas, idCanvas, detCanvas])
	vehicles.append(v)
	infoLabel.configure(text = infoListToText())

def infoListToText():
	txt = ""
	for i in range(len(vehicles)):
		v = vehicles[i]
		txt += "{}. speed = {:.1f}\n".format(i, v.getAngSpeed())
	return txt

def vehiclesMove():
	global timerCounter, info, totalLoops
	if move:
		env.step(vehicles)
		for v in vehicles:
			v.update()
			if (v.getLooped()): totalLoops += 1
			infoLabel.configure(text = infoListToText())
			canvas.coords(v.getVehicleCanvas(), *flatten(v.getVehPoints()))
			for i in range(len(v.getWheelsCanvas())):
				w = v.getWheelsCanvas()[i]
				wP = v.getWheelPoints()[i]
				canvas.coords(w, *flatten(wP))
			canvas.coords(v.getDirCanvas(), *flatten(v.getDirPoints()))
			canvas.coords(v.getIDCanvas(), v.getX(), v.getY())
			canvas.coords(v.getDetRadCanvas(), v.getX() - idm.DETECTION_DIST, 
				v.getY() - idm.DETECTION_DIST, v.getX() + idm.DETECTION_DIST, 
				v.getY() + idm.DETECTION_DIST)
		timerLabel.config(text = "{0:.1f} s".format(round(time.time() - timerCounter, 1)))
		loopLabel.config(text = totalLoops)
		stopTesting()
	root.after(10, vehiclesMove)

def stopTesting():
	global testing, timerCounter, testingTime, move, mode
	if (timerCounter != None and testing and 
		(time.time() - timerCounter) >= testingTime):
		move = False
		print(modeName(), totalLoops)
		if (mode == NON_COOP):
			root.after(1000, testCase(COOP))
			timerCounter = time.time()
		else: testing = False


def drawTrack(x, y, outR, inR):
	canvas.create_oval(x - outR, y - outR, x + outR, y + outR,
		fill = 'darkGreen', outline = "")
	canvas.create_oval(x - inR, y - inR, x + inR, y + inR,
		fill = 'white', outline = "")

def figure8Track():
	inR =  outR - TRACK_WIDTH
	drawTrack(trackLeftX, trackY, outR, inR)
	drawTrack(trackRightX, trackY, outR, inR)

	# Checkered finish line
	drawFinishLine(CANVAS_WIDTH // 2 - (TRACK_WIDTH // 2), trackY, 13)
	canvas.create_oval(trackRightX - inR, trackY - inR, trackRightX + inR,
		trackY + inR, fill = 'white', outline = "")

def drawFinishLine(x, y, blocks):
	d = TRACK_WIDTH // blocks
	for j in [-1, 0]:
		y1, y2 = y + d * j, y + d * (j + 1)
		for i in range(blocks + 1):
			x1 = x + i * d
			x2 = x + (i + 1) * d
			if ((i + j) % 2 == 0): color = 'white'
			else: color = 'black'
			canvas.create_rectangle(x1, y1, x2, y2, fill = color, outline = "")

def twoLaneTrack():
	drawTrack(trackX, trackY, outR + TRACK_WIDTH, outR - TRACK_WIDTH)
	canvas.create_oval(trackX - outR, trackY - outR, trackX + outR,
		trackY + outR, dash = (10, 7), outline = "white")

	# Checkered finish line
	drawFinishLine(trackX + outR - TRACK_WIDTH, trackY, 13)
	drawFinishLine(trackX + outR, trackY, 13)

def drawTrackButton():
	global trackBCoords
	x1, y1 = CANVAS_WIDTH - 2 * MARGIN, MARGIN // 4
	x2, y2 = CANVAS_WIDTH - MARGIN // 2, MARGIN // 2
	canvas.create_rectangle(x1, y1, x2, y2, fill = "PaleGreen1")
	canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2,
		text = "Change Track", fill = 'darkGreen')
	trackBCoords = [x1, y1, x2, y2]

def drawDetectionButton():
	global detBCoords
	x1, y1 = CANVAS_WIDTH - 2 * MARGIN, MARGIN // 2
	x2, y2 = CANVAS_WIDTH - MARGIN // 2, MARGIN * 3 // 4
	canvas.create_rectangle(x1, y1, x2, y2, fill = "tan1")
	canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2,
		text = "Detection Radius", fill = 'OrangeRed4')
	detBCoords = [x1, y1, x2, y2]

def drawModeButton():
	global modeBCoords
	x1, y1 = CANVAS_WIDTH - 2 * MARGIN, MARGIN * 3 // 4
	x2, y2 = CANVAS_WIDTH - MARGIN // 2, MARGIN
	canvas.create_rectangle(x1, y1, x2, y2, fill = "plum1")
	canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2,
		text = "Mode", fill = 'DarkOrchid4')
	modeBCoords = [x1, y1, x2, y2]

def drawTimer():
	global loopLabel, timerLabel
	loopLabel.configure(text = str(totalLoops))
	timerLabel.place(x = CANVAS_WIDTH/2 - MARGIN, y = MARGIN // 3 * 2)
	loopLabel.place(x = CANVAS_WIDTH/2 + MARGIN, y = MARGIN // 3 * 2)

def changeTrack(x, y):
	global trackBCoords, TWO_LANE
	[x1, y1, x2, y2] = trackBCoords
	if (x > x1 and x < x2 and y > y1 and y < y2):
		TWO_LANE = not TWO_LANE
		reset()

def addDetectionRadius(x, y):
	global detBCoords, detectionRadius
	[x1, y1, x2, y2] = detBCoords
	if (x > x1 and x < x2 and y > y1 and y < y2):
		detectionRadius = not detectionRadius
		for v in vehicles:
			if (detectionRadius): 
				canvas.itemconfig(v.getDetRadCanvas(), state = tk.NORMAL)
			else: canvas.itemconfig(v.getDetRadCanvas(), state = tk.HIDDEN)

def changeMode(x, y):
	global mode, modeBCoords, move
	[x1, y1, x2, y2] = modeBCoords
	if (x > x1 and x < x2 and y > y1 and y < y2):
		if (mode == NON_COOP): mode = COOP
		else: mode = NON_COOP
		modeLabel.configure(text = modeName())

def reset():
	global vehicles, totalLoops, mode
	totalLoops = 0

	initEnv()
	canvas.delete("all")
	# Add title and car information at top and bottom of screen
	canvas.create_text(CANVAS_WIDTH/2, MARGIN // 10,
		text='      Cooperative vs Non-Cooperative Autonomous Driving')
	canvas.create_text(CANVAS_WIDTH/2 - MARGIN * 5 // 6, MARGIN // 2,
		text='Timer:', fill = 'red')
	canvas.create_text(CANVAS_WIDTH/2 + MARGIN, MARGIN // 2,
		text='   Total Loops:', fill = 'darkBlue')
	vehicles = []

	infoLabel.place(x = MARGIN // 9, y = MARGIN // 9)
	infoLabel.configure(text = "")
	modeLabel.place(x = CANVAS_WIDTH / 2 - MARGIN // 3, y = MARGIN // 5)
	modeLabel.configure(text = modeName())
	drawTrackButton()
	drawDetectionButton()
	drawModeButton()
	drawTimer()
	pickTrack()

def pickTrack():
	if (TWO_LANE):
		twoLaneTrack()
	else:
		figure8Track()


if __name__ == "__main__":
	reset()
	drawTrackButton()

	# Pick track
	pickTrack()
	vehiclesMove()

	root.bind("<Key>", keyPress)
	root.bind("<Button-1>", mousePress)
	root.mainloop()
