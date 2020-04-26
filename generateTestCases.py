import random
import os
import argparse

from graphicsSim import isValidTest

NUM_CASES = 3
MIN_CARS = 9
MAX_CARS = 13
MAX_DEG = 360
TIME = 40
SEED = 11

def getArgs():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', required=False, type=str, help='Path to txt file for testing inputs')
    args = parser.parse_args()
    return args


def makeTestCase():
    numCars = random.randint(MIN_CARS, MAX_CARS)
    while True:
        testList = []
        for i in range(numCars):
            direction = random.randint(0,1)
            direction = -1 if direction == 0 else 1
            position = random.randint(0, MAX_DEG - 1)
            testList.append((position, direction))
        if isValidTest([0,testList]):
            break
    return testList


def generateAllCases():
    allTests = []
    for i in range(NUM_CASES):
        allTests.append([TIME, makeTestCase()])
    return allTests


def convertToStrings(tests):
    testStrings = []
    for test in tests:
        newString = ""
        newString += str(test[0]) + "."
        cars = test[1]
        strings = [" " + str(car[0]) + " " + str(car[1]) for car in cars]
        newString += " ;".join(strings) + "\n"
        testStrings.append(newString)
    return testStrings


def outputToFile(tests, path):
    if os.path.isfile(path):
        with open(path, 'a+') as file:
            file.writelines(tests)
    else:
        with open(path, 'w') as file:
            file.writelines(tests)


if __name__ == "__main__":
    random.seed(SEED)
    args = getArgs()
    tests = generateAllCases()
    testStrings = convertToStrings(tests)
    outputToFile(testStrings, args.i)
