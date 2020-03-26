from track import Track
from vehicle import Vehicle
import vehicle
import tkinter as tk
import itertools, math

root = tk.Tk()
TK_SILENCE_DEPRECATION=1

CANVAS_WIDTH = 1200
CANVAS_HEIGHT = 800
TRACK_WIDTH = vehicle.VEH_WIDTH * 4
MARGIN = 60

outR = (CANVAS_WIDTH + TRACK_WIDTH - 2 * MARGIN) // 4
vehR = outR - (TRACK_WIDTH // 2)

trackLeftX = MARGIN + outR
trackRightX = CANVAS_WIDTH - MARGIN - outR
trackY = CANVAS_HEIGHT // 2

canvas = tk.Canvas(root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT)
# x, y = 0, 0
# canvas.create_text(CANVAS_WIDTH/2, CANVAS_HEIGHT - MARGIN,
# 		text = str(x) + ', ' + str(y))
move = False
canvas.pack()

vehicles = []

def keyPress(event):
	global move
	if (event.char == "s"):
		move = not move

def mousePress(event):
	# global x, y
	x, y = event.x, event.y
	r, direc = inTrack(x, y)
	if (r != None):
		theta = placeOnTrack(x, y, direc, r)
		makeVehicle(theta, direc)
	# canvas.create_text(CANVAS_WIDTH/2, CANVAS_HEIGHT - MARGIN,
	# 	text = str(event.x) + ', ' + str(event.y))

def inTrack(x, y):
	# checks left track circle first
	r = math.sqrt((trackLeftX - x)**2 + (trackY - y)**2)
	if (r <= outR and r >= (outR - TRACK_WIDTH)): return (r, vehicle.LEFT)
	# then right track
	r = math.sqrt((trackRightX - x)**2 + (trackY - y)**2)
	if (r <= outR and r >= (outR - TRACK_WIDTH)): return (r, vehicle.RIGHT)
	return (None, None)

def placeOnTrack(x, y, direc, r):
	if (direc == vehicle.LEFT): x -= trackLeftX
	else: x -= trackRightX
	y -= trackY
	a = math.acos(float(x / r))
	if (y > 0): a *= -1.0
	return a

def flatten(l):
	return [item for tup in l for item in tup]

def makeVehicle(theta, direc):
	# Add Vehicle
	if (direc == vehicle.LEFT):
		veh = Vehicle(trackLeftX, trackY, vehR, theta, direc)
	else:
		veh = Vehicle(trackRightX, trackY, vehR, theta, direc)
	vehCanvas = canvas.create_polygon(veh.getVehPoints(), fill='red')
	wheelsCanvas = []
	for wP in veh.getWheelPoints():
		wheelsCanvas.append(canvas.create_polygon(wP, fill='black'))
	dirCanvas = canvas.create_polygon(veh.getDirPoints(), fill = 'yellow')
	vehicles.append((veh, vehCanvas, wheelsCanvas, dirCanvas))


def vehiclesMove():
	if move:
		for v, vehCanvas, wheelsCanvas, dirCanvas in vehicles:
			v.turn()
			canvas.coords(vehCanvas, *flatten(v.getVehPoints()))
			for i in range(len(wheelsCanvas)):
				w = wheelsCanvas[i]
				wP = v.getWheelPoints()[i]
				canvas.coords(w, *flatten(wP))
			canvas.coords(dirCanvas, *flatten(v.getDirPoints()))
	root.after(1, vehiclesMove)


def drawTrack(x, y, outR, inR):
	canvas.create_oval(x - outR, y - outR, x + outR, y + outR, 
		fill = 'darkGreen', outline = "")
	canvas.create_oval(x - inR, y - inR, x + inR, y + inR,
		fill = 'white', outline = "")



if __name__ == "__main__":
	canvas.create_text(CANVAS_WIDTH/2, MARGIN,
		text='Cooperative vs Non-Cooperative Autonomous Driving')

	# Add Track
	drawTrack(trackLeftX, trackY, outR, outR - TRACK_WIDTH)
	drawTrack(trackRightX, trackY, outR, outR - TRACK_WIDTH)

	# makeVehicle(0, vehicle.LEFT)
	# makeVehicle(math.pi / 2, vehicle.LEFT)
	# makeVehicle(math.pi / 2, vehicle.RIGHT)

	vehiclesMove()

	root.bind("<Key>", keyPress)
	root.bind("<Button-1>", mousePress)
	root.mainloop()
