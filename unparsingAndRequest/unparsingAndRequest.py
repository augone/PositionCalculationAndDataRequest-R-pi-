import threading
from time import ctime, sleep
import math
#UNPARSING CODE:
import Rpi.GPIO as GPIO
import time
import Serial
from array import *

i = 0
RADIUS = 6
WIDTH = 26
TLMAX = 3600
TRMAX = 3600
DT = 0.5 #double
pi = 3.14
#initial conditions
X = [0 for a in range(10)]
Y = [0 for a in range(10)]
Phi = [0 for a in range(10)]
# the following array is used but not defined in the original C code, so I added it here for initialization
del_Phi = [0 for a in range(10)]

# serial port initialization
ser = serial.Serial ( 
    port = '/dev/ttyAMAO',  
    baudrate = 115200,                 
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
    )
sum1=0
sum2=0
diff=0
theta=0

class Queue:
    def __init__(self):
        self.items = []

    def isEmpty(self):
        return len(self.items) == 0
    def push(self,item):
        self.items.append(item)
    def pop(self):
        if not self.isEmpty():
            return self.items.pop(0)
        else:
            return 
    def peek(self):
        if not self.isEmpty():
            return self.items[0]
    def size(self):
        return len(self.items)
    def copy(self):
        tempQueue = Queue(self.items)
        return tempQueue
    def clear(self):
        self.items = []


myQueue = Queue()
EncoderLeft = Queue()
EncoderRight = Queue()

AccelerometerX = Queue()
AccelerometerY = Queue()
AccelerometerZ = Queue()

MagnetometerX = Queue()
MagnetometerY = Queue()
MagnetometerZ = Queue()

GPSLatitudeDegree = Queue()
GPSLatitudeMinute = Queue()
GPSLatitudeDirection = Queue()
GPSLongitudeDegree = Queue()
GPSLongitudeMinute = Queue()
GPSLongitudeDirection = Queue()
GPSAltitude = Queue()
GPSSpeed = Queue()
GPSTime = Queue()

Temperature = Queue()

Timestamp = Queue()

myLock = threading.Lock()
rawStringQueue = Queue()



def requestData():
    global rawStringQueue,myLock
    
    # unparsing part
    ser.write ('H')
    #count = [[0]*4 for j in range(4)]
    #val = [ [0]*30 for j in range (30)]
    serialdata = ser.read(1)
    while serialdata != 'K':
        serialdata = ser.read(1)
    returnedString = ''
    requestFinished = False
    exclaimationCount = 0
    ser.write('A')
    ser.write('ELERAXAYAZMXMYMZGXGYGZGSGTTE')
    myLock.acquire()
    try:
        while not requestFinished:
            tempChar = ser.read(1)
            returnedString += tempChar
            if tempChar == '!':
                exclaimationCount += 1
                if exclaimationCount == 2:
                    requestFinished = True
    finally:
        myLock.release()
        rawStringQueue.push(returnedString)
        timer = threading.Timer(0.1,requestData)
    

def unparsing():
    global rawStringQueue,mylock
    while True:
        while rawStringQueue.isEmpty():
            pass

        mylock.acquire()
        try:
            copyOfrawStringQueue = rawStringQueue.copy()
            rawStringQueue.clear()
        finally:
            myLock.release()
    
        
        while not copyOfrawStringQueue.isempty():
            charactercounter = 0
            tempstr = copyOfrawStringQueue.pop()
            strtoken = tempstr[charactercounter:charactercounter+2]
            while strtoken != '!!':
                charactercounter += 2
                if strtoken == 'EL':
                    left = charactercounter
                    right = charactercounter
                    for i in range(charactercounter,len(tempstr)):
                            
                        if not tempstr[i].isdigit():
                               
                            right = i
                            break

                    #print(tempstr[left:right],'watchpoint3')
                    EncoderLeft.push(int(tempstr[left:right]))
                    charactercounter = right
                elif strtoken == 'ER':
                    left = charactercounter
                    right = charactercounter
                    for i in range(charactercounter,len(tempstr)):
                            
                        if not tempstr[i].isdigit():
                      
                            right = i
                            break

                    EncoderRight.push(int(tempstr[left:right]))
                    charactercounter = right
                elif strtoken == 'AX':
                    left = charactercounter
                    right = charactercounter
                    for i in range(charactercounter,len(tempstr)):
                            
                        if(tempstr[i].isalpha() or tempstr[i] == '!'):
                            right = i
                            break


                    AccelerometerX.push(float(tempstr[left:right]))
                    charactercounter = right
                elif strtoken == 'AY':
                    left = charactercounter
                    right = charactercounter
                    for i in range(charactercounter,len(tempstr)):
                            
                        if(tempstr[i].isalpha() or tempstr[i] == '!'):
                            right = i
                            break    


                    AccelerometerY.push(float(tempstr[left:right]))
                    charactercounter = right
                elif strtoken == 'AZ':
                    left = charactercounter
                    right = charactercounter
                    for i in range(charactercounter,len(tempstr)):
                            
                        if(tempstr[i].isalpha() or tempstr[i] == '!'):
                            right = i
                            break    


                    AccelerometerZ.push(float(tempstr[left:right]))
                    charactercounter = right
                elif strtoken == 'MX':
                    left = charactercounter
                    right = charactercounter
                    for i in range(charactercounter,len(tempstr)):
                            
                        if(tempstr[i].isalpha() or tempstr[i] == '!'):
                            right = i
                            break    


                    MagnetometerX.push(float(tempstr[left:right]))
                    charactercounter = right
                elif strtoken == 'MY':
                    left = charactercounter
                    right = charactercounter
                    for i in range(charactercounter,len(tempstr)):
                            
                        if(tempstr[i].isalpha() or tempstr[i] == '!'):
                            right = i
                            break    


                    MagnetometerY.push(float(tempstr[left:right]))
                    charactercounter = right

                elif strtoken == 'MZ':
                    left = charactercounter
                    right = charactercounter
                    for i in range(charactercounter,len(tempstr)):
                            
                        if(tempstr[i].isalpha() or tempstr[i] == '!'):
                            right = i
                            break

                    MagnetometerZ.push(float(tempstr[left:right]))
                    charactercounter = right
                elif strtoken == 'GX':
                    degreeLen = 2
                    GPSLatitudeDegree.push(int(tempstr[charactercounter:charactercounter+degreeLen]))
                    GPSLatitudeMinute.push(float(tempstr[charactercounter+degreeLen:charactercounter+degreeLen+6]))
                        
                    charactercounter += degreeLen+6
                    GPSLatitudeDirection.push(tempstr[charactercounter])
                    charactercounter =charactercounter+1
                elif strtoken == 'GY':
                    degreeLen = 3
                    GPSLongitudeDegree.push(int(tempstr[charactercounter:charactercounter+degreeLen]))
                    GPSLongitudeMinute.push(float(tempstr[charactercounter+degreeLen:charactercounter+degreeLen+6]))
                        
                    charactercounter += degreeLen+6
                    GPSLongitudeDirection.push(tempstr[charactercounter])
                    charactercounter =charactercounter+1
                elif strtoken == 'GZ':
                    left = charactercounter
                    right = charactercounter
                    for i in range(charactercounter,len(tempstr)):
                            
                        if(tempstr[i].isalpha() or tempstr[i] == '!'):
                            right = i
                            break

                    GPSAltitude.push(float(tempstr[left:right]))
                    charactercounter = right
                elif strtoken == 'GS':
                    left = charactercounter
                    right = charactercounter
                    for i in range(charactercounter,len(tempstr)):
                            
                        if(tempstr[i].isalpha() or tempstr[i] == '!'):
                            right = i

                            break
                    GPSSpeed.push(float(tempstr[left:right]))
                    charactercounter = right
                elif strtoken == 'GT':
                    pass #Time format is unknow, might be xx.xx.xx
                    #GPSTime.push(tempstr[charactercounter:charactercounter+8])

                elif strtoken == 'TE':
                    left = charactercounter
                    right = charactercounter
                    for i in range(charactercounter,len(tempstr)):
                            
                        if(tempstr[i].isalpha() or tempstr[i] == '!'):
                            right = i
                            break

                    Temperature.push(float(tempstr[left:right]))
                    charactercounter = right
                elif strtoken == 'TS':
                    Timestamp.push(tempstr[charactercounter:charactercounter+8])
                    charactercounter = charactercounter + 8
                strtoken = tempstr[charactercounter:charactercounter+2]

        control()

        
        




def control(func):
    global myLock,EncoderLeft,EncoderRight
    i = 1
    if i == 10:
        # If array size limit reached save the last data as zeroth element and continue
        X[0] = X[9]
        Y[0] = Y[9]
        Phi[0] = Phi[9]
        i = 1
    if EncoderLeft.isEmpty():
        pass
    else:
        while not EncoderLeft.isEmpty():
            #This is removing the offset. An reading of 5000 means 0 ticks, 4000 means 1000 ticks in reverse direction, 6000
            #vice versa
            ticks_left = 5000 - EncoderLeft.pop() #assuming Queue stores right,left,.....
            ticks_right = 5000 - EncoderRight.pop()

            SL = ((2*pi*RADIUS*ticks_left)/TLMAX)
            SR = ((2*pi*RADIUS*ticks_right)/TRMAX)
            if SL != SR:
                K = (SL + SR)/(SL - SR)
                del_Phi[i] = ((SL - SR)/WIDTH)# what is del_Phi?
                Phi[i] = del_Phi[i] + Phi[i-1]
                X[i] = X[i-1] - ((WIDTH/2)*K*(math.cos(Phi[i]) - math.cos(Phi[i-1])))
                Y[i] = Y[i-1] + ((WIDTH/2)*K*(math.sin(Phi[i]) - math.sin(Phi[i-1])))

            if SL == SR:
                Phi[i] = Phi[i-1]
                X[i] = X[i-1] + ((((SL+SR)/2)*DT)*(math.sin(Phi[i])))
                Y[i] = Y[i-1] + ((((SL+SR)/2)*DT)*(math.cos(Phi[i])))

            i += 1

        

threads = []
t1 = threading.Thread(target = unparsing, args = ('requestData',))
threads.append(t1)

#t1 = threading.Thread(target = unparsing, args = ('unparsing',))
#threads.append(t1)
t2 = threading.Thread(target = control, args = ('unparsing',))#and control
threads.append(t2)
#t3 = threading.Thread(target = requestData, args = ('requestData',))
#threads.append(t2)
#def noFunc():
#    pass
#Timer = threading.Timer(0.1,noFunc)

if __name__ == '__main__':

    for t in threads:
        #t.setDaemon(False)
        t.start()



