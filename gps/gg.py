import serial
import pynmea2


def parseGPS(s):
    if s.find('GGA') > -1:
        msg = pynmea2.parse(s)
        print("Timestamp: %s -- Lat: %s %s -- Lon: %s %s -- Altitude: %s %s" % (
            msg.timestamp, msg.lat, msg.lat_dir, msg.lon, msg.lon_dir, msg.altitude, msg.altitude_units))


serialPort = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.5)
while True:
    s = serialPort.readline()
    # print(s)
    # print(type(s.decode()))
    # print(s.find(b'GGA'))
    s = s.decode()
    parseGPS(s)
