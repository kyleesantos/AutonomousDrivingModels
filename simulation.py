import argparse
import csv

from graphics import runGraphics

fields = ['Non-Cooperative', 'Cooperative', 'Time']

def getArgs():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', required=False, type=str, help='Path to txt file for testing inputs')
    parser.add_argument('-o', required=False, type=str, help='Path to txt file for writing outputs')
    args = parser.parse_args()
    return args

def parseInput(path):
    testCases = []
    with open(path, 'r') as file:
        for line in file:
            line = line.strip()
            if not line or line.startswith('#'): continue
            timeCars = line.split('.')
            time = float(timeCars[0].strip())
            cars = [word.strip() for word in timeCars[1].strip().split(';')]
            test = [(int(car.split(' ')[0]), int(car.split(' ')[1])) for car in cars]
            testCases.append((time, test))
    return testCases

def outputResults(results, path):
    with open(path, 'w') as file:
        csvWriter = csv.writer(file)
        csvWriter.writerow(fields)
        results = [[str(a),str(b),str(c)] for (a,b,c) in results]
        csvWriter.writerows(results)


def main():
    args = getArgs()
    if args.i and args.o:
        testList = parseInput(args.i)
        print(testList)
        results = [(12,12,13), (14,15,16)]
        results = runGraphics(tFlag=True, tLists=testList)
        outputResults(results, args.o)
    else:
        runGraphics()

if __name__ == '__main__':
    main()
