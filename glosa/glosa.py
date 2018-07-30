#!/usr/bin/env python

import os
import sys
import optparse
import subprocess
import numpy as np
import math
from time import sleep

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
# the port used for communicating with your sumo instance
PORT = 8873

def run():
    """execute the TraCI control loop"""
    traci.init(PORT)
    positionJunctionE0 = traci.lane.getLength("gneE0_0")
    positionJunctionE7 = traci.lane.getLength("gneE7_0")
    positionJunctionE6 = traci.lane.getLength("gneE6_0")
    positionJunctionE8 = traci.lane.getLength("gneE8_0")


    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        veh_ids_E0_0 = traci.lane.getLastStepVehicleIDs("gneE0_0")
        veh_ids1_E0_1 = traci.lane.getLastStepVehicleIDs("gneE0_1")
        veh_ids1_E7_0 = traci.lane.getLastStepVehicleIDs("gneE7_0")
        veh_ids1_E7_1 = traci.lane.getLastStepVehicleIDs("gneE7_1")
        veh_ids1_E6_0 = traci.lane.getLastStepVehicleIDs("gneE6_0")
        veh_ids1_E6_1 = traci.lane.getLastStepVehicleIDs("gneE6_1")
        veh_ids1_E8_0 = traci.lane.getLastStepVehicleIDs("gneE8_0")
        veh_ids1_E8_1 = traci.lane.getLastStepVehicleIDs("gneE8_1")


        caller(veh_ids_E0_0,positionJunctionE0)
        caller(veh_ids1_E0_1, positionJunctionE0)
        caller(veh_ids1_E7_0, positionJunctionE7)
        caller(veh_ids1_E7_1, positionJunctionE7)
        caller1(veh_ids1_E6_0, positionJunctionE6)
        caller1(veh_ids1_E6_1, positionJunctionE6)
        caller1(veh_ids1_E8_0, positionJunctionE8)
        caller1(veh_ids1_E8_1, positionJunctionE8)

    traci.close()
    sys.stdout.flush()


def caller(veh_ids,positionJunctionX):
    arraySize = len(veh_ids)
    for x in range(0, arraySize):
        print("vehicle Id", veh_ids[x])
        xPosition, yPosition = traci.vehicle.getPosition(veh_ids[x])
        distance = positionJunctionX - xPosition
        # if distance > 240:
        #    status[veh_ids[x]] = 0
        phrase = traci.trafficlight.getRedYellowGreenState("gneJ1")
        # get vehid  setup outsde func
        if phrase == "rrrrGGGgrrrrGGGg":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 1<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 1
            algo(veh_ids[x], positionJunctionX, phraseNo)

        elif phrase == "rrrryyygrrrryyyg":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 2<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 2
            algo(veh_ids[x], positionJunctionX, phraseNo)

        elif phrase == "rrrrrrrGrrrrrrrG":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 3<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 3
            algo(veh_ids[x], positionJunctionX, phraseNo)

        elif phrase == "rrrrrrryrrrrrrry":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 4<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 4
            algo(veh_ids[x], positionJunctionX, phraseNo)

        elif phrase == "GGGgrrrrGGGgrrrr":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 5<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 5
            algo(veh_ids[x], positionJunctionX, phraseNo)

        elif phrase == "yyygrrrryyygrrrr":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 6<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 6
            algo(veh_ids[x], positionJunctionX, phraseNo)

        elif phrase == "rrrGrrrrrrrGrrrr":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 7<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 7
            algo(veh_ids[x], positionJunctionX, phraseNo)

        elif phrase == "rrryrrrrrrryrrrr":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 8<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 8
            algo(veh_ids[x], positionJunctionX, phraseNo)
    print "-----------------------------------------------------------------------------------------------------------------------------"


def caller1(veh_ids,positionJunctionX):
    arraySize = len(veh_ids)
    for x in range(0, arraySize):
        print ("vehicle Id", veh_ids[x])
        xPosition, yPosition = traci.vehicle.getPosition(veh_ids[x])
        distance = positionJunctionX - xPosition
        # if distance > 240:
        #    status[veh_ids[x]] = 0
        phrase = traci.trafficlight.getRedYellowGreenState("gneJ1")
        # get vehid  setup outsde func
        if phrase == "GGGgrrrrGGGgrrrr":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 1<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 1
            algo(veh_ids[x], positionJunctionX, phraseNo)

        elif phrase == "yyygrrrryyygrrrr":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 2<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 2
            algo(veh_ids[x], positionJunctionX, phraseNo)

        elif phrase == "rrrGrrrrrrrGrrrr":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 3<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 3
            algo(veh_ids[x], positionJunctionX, phraseNo)

        elif phrase == "rrryrrrrrrryrrrr":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 4<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 4
            algo(veh_ids[x], positionJunctionX, phraseNo)

        elif phrase == "rrrrGGGgrrrrGGGg":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 5<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 5
            algo(veh_ids[x], positionJunctionX, phraseNo)

        elif phrase == "rrrryyygrrrryyyg":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 6<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 6
            algo(veh_ids[x], positionJunctionX, phraseNo)

        elif phrase == "rrrrrrrGrrrrrrrG":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 7<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 7
            algo(veh_ids[x], positionJunctionX, phraseNo)

        elif phrase == "rrrrrrryrrrrrrry":
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHRASE 8<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
            phraseNo = 8
            algo(veh_ids[x], positionJunctionX, phraseNo)
    print "-----------------------------------------------------------------------------------------------------------------------------"



def algo(veh_id,positionJunctionX, phraseNo):

    MAX_SPEED = 14
    status = {}
    arrival_time = 0

    dmax = 0
    tmax = 0
    arrivalMargin = 2
    ignoreRedLight = 31 - 16 - 1


    print (traci.trafficlight.getNextSwitch("gneJ1") - traci.simulation.getCurrentTime()) / 1000

    arrival_time = 0
    # print traci.trafficlight.getCompleteRedYellowGreenDefinition("gneJ1")
    # if veh_ids[x] != "flow1.1" : continue
    xPosition = traci.vehicle.getLanePosition(veh_id)
    speed = traci.vehicle.getSpeed(veh_id)
    distance = positionJunctionX - xPosition
    acceleration = traci.vehicle.getAcceleration(veh_id)
    traci.vehicle.setSpeedMode(veh_id, ignoreRedLight)

    if acceleration != 0:
        tmax = (MAX_SPEED - speed) / acceleration
        dmax = tmax * ((MAX_SPEED + speed) / 2)
        print "length of the track",positionJunctionX
        print "tmax", tmax
        print "dmax", dmax
        if dmax > positionJunctionX:
            arrival_time = -(speed / acceleration) + math.sqrt(
                    ((speed * speed) / (acceleration * acceleration)) + ((2 * distance) / acceleration))
            print "executed1"
            # print arrival_time
        else:
            arrival_time = tmax + ((positionJunctionX - dmax) / MAX_SPEED)
            # print "executed"
            print "executed2"
            print arrival_time

    else :
        if speed != 0:
            arrival_time = distance / speed
            # print arrival_time
            print "executed3"

    remaining = (traci.trafficlight.getNextSwitch("gneJ1") - traci.simulation.getCurrentTime()) / 1000

    print "remaining time", remaining
    print "speed", speed
    print "distance", distance
    print "pos", xPosition
    print "posJ", positionJunctionX
    print "xPosition", xPosition
    print "accel", acceleration
    sys.stdout.flush()

    print "arrival timeeeeeeeeeeeee", arrival_time
    if phraseNo == 1:
        if np.greater(remaining, arrival_time) and ((remaining - arrival_time) >= arrivalMargin):
            traci.vehicle.setSpeed(veh_id, MAX_SPEED)
            # traci.vehicle.slowDown(veh_ids[x], MAX_SPEED, (arrival_time * 1000))
            # traci.vehicle.setColor(veh_ids[x],(244,95,66,0))
            print "MAX SPEED ", veh_id

        else:
            nextGreenPhrase = 59 + remaining + arrivalMargin
            expectedSpeed = max(0, ((2 * distance) / nextGreenPhrase) - speed)
            traci.vehicle.slowDown(veh_id, expectedSpeed, (nextGreenPhrase * 1000))
            print "Arrive in NEXT GREEN PHRASE", nextGreenPhrase
            status[veh_id] = 1

    else:
        print "positionnnnnnnn", xPosition
        print "statussssssssss", status
        #if status[veh_ids[x]] != 1:
        if phraseNo ==1 :
            nextGreenPhrase = 59 + remaining + arrivalMargin
        elif phraseNo ==2:
            nextGreenPhrase = 55 + remaining + arrivalMargin
        elif phraseNo ==3:
            nextGreenPhrase = 49 + remaining + arrivalMargin
        elif phraseNo ==4:
            nextGreenPhrase = 45 + remaining + arrivalMargin
        elif phraseNo ==5:
            nextGreenPhrase = 14 + remaining + arrivalMargin
        elif phraseNo ==6:
            nextGreenPhrase = 10 + remaining + arrivalMargin
        elif phraseNo == 7:
            nextGreenPhrase = 4+remaining + arrivalMargin
        elif phraseNo == 8:
            nextGreenPhrase = (remaining) + arrivalMargin

        if nextGreenPhrase != 0:
            expectedSpeed = max(0, ((2 * distance) / nextGreenPhrase) - speed)
        else:
            expectedSpeed = speed

        print "expected speed ", expectedSpeed
        traci.vehicle.slowDown(veh_id, expectedSpeed, (nextGreenPhrase * 1000))
        print "Arrive in NEXT GREEN PHRASE", nextGreenPhrase
        status[veh_id] = 1

    print "new speed", traci.vehicle.getSpeed(veh_id)
    print "***************************"




def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

 
    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    sumoProcess = subprocess.Popen([sumoBinary, "-c", "demo.sumo.cfg", "--tripinfo-output",
                                    "tripinfo.xml", "--remote-port", str(PORT)], stdout=sys.stdout, stderr=sys.stderr)
    run()
    sumoProcess.wait()
