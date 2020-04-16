import argparse
import csv
import os

from graphics import runGraphics

fields = ['Time','Coop Total Loops', 'Coop Average Velocity', 'Coop Average Acceleration',
            'Coop Average Deceleration', 'Coop Waiting Time', 'Non-Coop Total Loops',
            'Non-Coop Average Velocity', 'Non-Coop Average Acceleration',
            'Non-Coop Average Deceleration', 'Non-Coops Waiting Time', ]

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

def toString(lists):
    results = []
    for tuples in lists:
        tuplesResults = []
        for word in tuples:
            tuplesResults.append(str(word))
        results.append(tuplesResults)
    return results

def outputResults(results, path):
    if os.path.isfile(path):
        with open(path, 'a+') as file:
            csvWriter = csv.writer(file)
            csvWriter.writerows(results)
    else:
        with open(path, 'w') as file:
            csvWriter = csv.writer(file)
            csvWriter.writerow(fields)
            csvWriter.writerows(results)


def main():
    args = getArgs()
    if args.i and args.o:
        testList = parseInput(args.i)
        results = runGraphics(tFlag=True, tLists=testList)
        results = toString(results)
        outputResults(results, args.o)
    else:
        runGraphics()

if __name__ == '__main__':
    main()
