#!/usr/bin/python
import serial
import argparse
import time

if __name__ == "__main__":
    # Use argparse to get the serial port
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    args = parser.parse_args()

    # Open the serial port
    s = serial.Serial()
    s.port = args.port
    s.baudrate = 115200
    print "PYTHON: opening port"
    s.open()

    print "PYTHON: searching for start of scandata"
    while True:
        line = s.readline()
        #print "MICRO: {}".format(line),
        if "STATE_PILLAR_SCAN" in line:
            break


    print "PYTHON: found start of scandata"

    f = open("scandata.csv", "w")
    while True:
        line = s.readline()
        print "    {}".format(line),

        #print line to csv file

        if "STATE_DONE" in line:
            break
        line = ",".join(line.split(' '))
        f.write(line)
    print "PYTHON: finished grabbing scandata"

    f.close()

    s.close()
