import serial
import pynmea2

serialPort = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.5)

def GetGPS():
    s = serialPort.readline()
    # print(s)
    # print(type(s.decode()))
    # print(s.find(b'GGA'))
    s = s.decode()
    if s.find('GGA') > -1:
        msg = pynmea2.parse(s)
        lat=msg.lat
        lon=msg.lon
        altitude=msg.altitude
    return(lat,lon,altitude)

