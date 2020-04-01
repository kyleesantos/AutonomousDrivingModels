
import tkinter as tk
import itertools, math, time
import numpy as np

from cooperativePlanning import Coop_Env
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

counter = 0
timerLabel = Label(text = "0")
info = Label(text = "")
info.place(x = MARGIN // 9, y = MARGIN // 9)
buttonTrack = []
move = False
# array of veh object, veh canvas, wheels canvas, triangle canvas
vehicles = []
tempSpace = 20

coop_env = Coop_Env()
coop_env.setIntersection((601, 367))  # y was 397 initially
coop_env.setIntersectionThreshold(159) # threshold was 194 initially
coop_env.setWeights(np.array([0,1]))

def keyPress(event):
	global move, counter
	if (event.char == "s"):
		counter = time.time()
		move = not move
	if (event.char == "r"):
		counter = time.time()
		timerLabel.config(text = "0.0 s")
		reset()
	if (event.char == "1"):
		for v, _, _, _ in vehicles:
			v.increaseAngSpeed(1)
	if (event.char == "2"):
		for v, _, _, _ in vehicles:
			v.decreaseAngSpeed(1)

def mousePress(event):
	x, y = event.x, event.y
	changeTrack(x, y)
	r, direc = inTrack(x, y)
	if (r != None):
		theta = placeOnTrack(x, y, direc, r)
		makeVehicle(theta, direc)

def inTrack(x, y):
	# Check for overlap
	dir, tX, tY = vehicle.RIGHT, trackLeftX, trackY
	if (TWO_LANE): tX = trackX
	elif (x > CANVAS_WIDTH // 2): dir, tX = vehicle.LEFT, trackRightX
	for (v, _, _, _) in vehicles:
		x1 = x - tX
		y1 = y - tY
		r = math.sqrt((x1)**2 + (y1)**2)
		a = toDegrees(math.acos(float(x1 / r)))
		if (y1 > 0): a = (a * -1.0) % 360
		checkV = vehicle.Vehicle(tX, tY, vehR, a, dir, -1)
		if (vehiclesCollide(v, checkV)): return (None, None)

	# Check for 2 lane track first
	r = math.sqrt((tX - x)**2 + (tY - y)**2)
	if (TWO_LANE or dir == vehicle.RIGHT):
		if (r <= outR and r >= (outR - TRACK_WIDTH)): return (r, vehicle.RIGHT)
	elif (r <= outR and r >= (outR - TRACK_WIDTH)): return (r, vehicle.LEFT)
	return (None, None)

def placeOnTrack(x, y, direc, r):
	if (TWO_LANE): x -= trackX
	elif (direc == vehicle.RIGHT): x -= trackLeftX
	else: x -= trackRightX
	y -= trackY
	a = toDegrees(math.acos(float(x / r)))
	if (y > 0): a = (a * -1.0)
	return a

def flatten(l):
	return [item for tup in l for item in tup]

def makeVehicle(theta, direc):
	global info
	# Add Vehicle
	if (TWO_LANE): tX = trackX
	elif (direc == vehicle.LEFT): tX = trackRightX
	else: tX = trackLeftX

	veh = vehicle.Vehicle(tX, trackY, vehR, theta, direc, len(vehicles))
	vehCanvas = canvas.create_polygon(veh.getVehPoints(), fill='red')
	wheelsCanvas = []
	for wP in veh.getWheelPoints():
		wheelsCanvas.append(canvas.create_polygon(wP, fill='black'))
	dirCanvas = canvas.create_polygon(veh.getDirPoints(), fill = 'yellow')
	vehicles.append((veh, vehCanvas, wheelsCanvas, dirCanvas))
	info.configure(text = infoListToText())

def infoListToText():
	txt = ""
	for i in range(len(vehicles)):
		(v, _, _, _) = vehicles[i]
		txt += "{}. speed = {:.1f}\n".format(i, v.getAngSpeed())
	return txt

def vehiclesMove():
	global counter, info
	if move:
		cars = [vehicle[0] for vehicle in vehicles]
		coop_env.step(cars)
		idm.updateAccels(cars, coop_env.getRightOfWay())
		for v, vehCanvas, wheelsCanvas, dirCanvas in vehicles:
			v.update()
			info.configure(text = infoListToText())
			canvas.coords(vehCanvas, *flatten(v.getVehPoints()))
			for i in range(len(wheelsCanvas)):
				w = wheelsCanvas[i]
				wP = v.getWheelPoints()[i]
				canvas.coords(w, *flatten(wP))
			canvas.coords(dirCanvas, *flatten(v.getDirPoints()))
		timerLabel.config(text = "{0:.1f} s".format(round(time.time() - counter, 1)))
	root.after(10, vehiclesMove)


def drawTrack(x, y, outR, inR):
	canvas.create_oval(x - outR, y - outR, x + outR, y + outR,
		fill = 'darkGreen', outline = "")
	canvas.create_oval(x - inR, y - inR, x + inR, y + inR,
		fill = 'white', outline = "")

def figure8Track():
	drawTrack(trackLeftX, trackY, outR, outR - TRACK_WIDTH)
	drawTrack(trackRightX, trackY, outR, outR - TRACK_WIDTH)

def twoLaneTrack():
	drawTrack(trackX, trackY, outR + TRACK_WIDTH, outR - TRACK_WIDTH)
	canvas.create_oval(trackX - outR, trackY - outR, trackX + outR,
		trackY + outR, dash = (10, 7), outline = "white")

def drawTrackButton():
	global buttonTrack
	x1, y1 = CANVAS_WIDTH - 2 * MARGIN, MARGIN // 4
	x2, y2 = CANVAS_WIDTH - MARGIN // 2, MARGIN // 2
	canvas.create_rectangle(x1, y1, x2, y2, fill = "grey",
		outline = "darkGreen")
	canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2,
		text = "Change Track", fill = 'darkGreen')
	buttonTrack = [x1, y1, x2, y2]

def changeTrack(x, y):
	global buttonTrack, TWO_LANE
	[x1, y1, x2, y2] = buttonTrack
	if (x > x1 and x < x2 and y > y1 and y < y2):
		TWO_LANE = not TWO_LANE
		reset()

def reset():
	global vehicles
	canvas.delete("all")
	# Add title and car information at top and bottom of screen
	canvas.create_text(CANVAS_WIDTH/2, MARGIN // 10,
		text='Cooperative vs Non-Cooperative Autonomous Driving')
	vehicles = []
	info.configure(text = "")
	drawTrackButton()
	pickTrack()

def pickTrack():
	if (TWO_LANE):
		twoLaneTrack()
	else:
		figure8Track()


if __name__ == "__main__":

	# timerLabel.place(x = CANVAS_WIDTH/2, y = MARGIN // 3 * 2)
	reset()
	drawTrackButton()

	# Pick track
	pickTrack()
	vehiclesMove()

	root.bind("<Key>", keyPress)
	root.bind("<Button-1>", mousePress)
	root.mainloop()
