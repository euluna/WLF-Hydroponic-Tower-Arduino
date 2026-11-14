# Include a copy of this script inside each sensor's folder for automatic data collection (eg. inside BH1750_LightSensor).
# Upload Arduino sensor scripts as usual, then CLOSE the Arduino IDE and run this script instead (otherwise, may get an access denied error).
# When starting this script for the first time, ensure closeTime has not already occurred for the current date (also, consider daylight savings).

import serial # pip install pyserial
from datetime import datetime, timedelta# for time and date related file naming, organization

arduino_port = "COM6" #feel free to adjust
baud = 9600
currTime = datetime.now() # exact current system time
now = datetime(currTime.year, currTime.month, currTime.day, currTime.hour, currTime.minute) # approx current time (by hour, min)
fileInterval = timedelta(days=1) # interval between new file save and creation (1 day by default)
closeTime = datetime(currTime.year, currTime.month, currTime.day, 23, 59) #today's close time, last two digits are hour, min in 24 hr time
# "close time" is the time when the currently open file will be saved, and a new file will be created to record data until the next fileInterval

fileName = "sensorData" + currTime.strftime("%Y-%m-%d") + ".csv"

serConnect = serial.Serial(arduino_port, baud, timeout=1) #establish serial connection
print("Connected to port: " + arduino_port)
file = open(fileName, "w")
print("Created file " + fileName)

def new_day_file():
    global closeTime
    global file
    global fileName
    # update closeTime
    closeTime = closeTime + fileInterval
    newFileName = "sensorData" + closeTime.strftime("%Y-%m-%d") + ".csv" # the new file name is labeled with its end date
    file.close()
    file = open(newFileName, "w")
    print("Created new file " + newFileName)
    fileName = newFileName

while True:
    serialData = str(serConnect.readline().decode('utf-8'))
    print(serialData)

    file=open(fileName, "a")
    file.write(serialData)
    currTime = datetime.now()
    now = datetime(currTime.year, currTime.month, currTime.day, currTime.hour, currTime.minute)
    if now == closeTime:
        new_day_file()