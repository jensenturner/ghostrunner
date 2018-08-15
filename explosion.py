#!/usr/bin/python
#from __future__ import absolute_import, division, print_function, unicode_literals

import math
print("imported math")

import time
print("imported time")
import numpy
print("imported numpy")
import smbus
print('imported smbus')
from gps import *
print('imported gps')
import threading
print('imported threading')
import struct
print('imported struct')
import pigpio
print('imported pigpio')

import pi3d
print("imported pi3d")

#some random vector utility functions for calculating matrices
def dot(a, b):
  return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def addV(a, b):
  return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]
def subV(a, b):
  return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]
def multV(v, a):
  return [v[0]*a, v[1]*a, v[2]*a]
def mag(a):
  return math.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])
def normalize(a):
  temp = mag(a)
  return [a[0]/temp, a[1]/temp, a[2]/temp]
def cross(a, b):
  return [a[1]*b[2] - a[2]*b[1], -a[0]*b[2]+a[2]*b[0], a[0]*b[1]-a[1]*b[0]]
def lerpV(a, b, weight):
  return [a[0]*(1-weight)+b[0]*weight, a[1]*(1-weight)+b[1]*weight, a[2]*(1-weight)+b[2]*weight]

def asymFrust(Sll, Slr, Sul, Epos, nearP, farP):
  #make the direction vectors for the screen orientation
  right = normalize(subV(Slr, Sll));
  up = normalize(subV(Sul, Sll));
  out = normalize(cross(right, up));

  EtoSll = subV(Sll, Epos);
  EtoSlr = subV(Slr, Epos);
  EtoSul = subV(Sul, Epos);
	
  #no minus sign on this because -z axis is forwards, so this being negative will cancel
  #with the negative near plane, making the right side on the right and left on the left
  d = dot(EtoSll, out);

  l = dot(right, EtoSll) * nearP / d;
  r = dot(right, EtoSlr) * nearP / d;
  t = dot(up, EtoSul) * nearP / d;
  b = dot(up, EtoSll) * nearP / d;

  #column vectors, -z axis forwards
  #struct mat4 pMat = {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
  pMat = numpy.zeros((4,4), dtype='float32')
  pMat[0][0] = 2 * nearP / (r - l);
  pMat[1][1] = 2 * nearP / (t - b);
  pMat[2][0] = -1 * (r + l) / (r - l);
  pMat[2][1] = -1 * (t + b) / (t - b);
  pMat[2][2] = 1 * (farP + nearP) / (farP - nearP);
  pMat[2][3] = 1;
  pMat[3][2] = -1 * (2 * farP*nearP) / (farP - nearP);
  #remember, old z value is transferred to w. make sure not to add 1 for no reason
  pMat[3][3] = 0;
	
  #now make the fancy rotation matrix
  M = [[right[0], right[1], right[2], 0],
       [up[0], up[1], up[2], 0],
       [out[0], out[1], out[2], 0],
       [0, 0, 0, 1]];

  #now do the fancy translation matrix
  #T = transmat(-Epos.x, -Epos.y, -Epos.z);

  #get the order right
  #return matMult(T, matMult(M, pMat));
  return pMat

#a utility function to make a rotation matrix
def LookAtMatrix(at, eye, up=[0,1,0]):
  zaxis = normalize(subV(at, eye))
  xaxis = normalize(cross(up, zaxis))
  yaxis = cross(zaxis, xaxis)
  xaxis.append(-dot(xaxis, eye))
  yaxis.append(-dot(yaxis, eye))
  zaxis.append(-dot(zaxis, eye))
  z = [0,0,0,1.0]
  return numpy.array([[xaxis[a], yaxis[a], zaxis[a], z[a]] for a in range(4)],
                     dtype='float32')

#magic that rotates one vector around another (as the axis)
def vecRotVec(vec, axis, angle):
  vecPara = multV(axis, dot(vec,axis)/dot(axis,axis))
  vecOrth = subV(vec, vecPara)
  orthMag = mag(vecOrth)
  normVec = cross(axis, vecOrth)
  vecRotOrth = multV(addV(multV(vecOrth, math.cos(angle)/orthMag), multV(normVec, math.sin(angle)/mag(normVec))), orthMag)
  return addV(vecRotOrth, vecPara)

#calculates the worldspace difference between two gps coordinates
def coordDif(cur, prev):
  zOff = (cur[0] - prev[0])/100.0*3.1415*earthR / 180.0;
  xOff = (cur[1] - prev[1])*3.1415*earthR*math.cos((prev[0] + cur[0]) * 3.1415 / 180.0 / 100)/180.0/10.0;
  return (xOff, 0, zOff)

#converts raw accelerometer data to angles (uncomment gyro lines and fix errors to add gyro support)
def convertAcc(accRaw):
  #gyroData = [*(struct.unpack('h', bytes(accRaw[0:2])) +
   #            struct.unpack('h', bytes(accRaw[2:4])) +
    #           struct.unpack('h', bytes(accRaw[4:6])))]
  accData = [*(struct.unpack('h', bytes(accRaw[0:2])) +
               struct.unpack('h', bytes(accRaw[2:4])) +
               struct.unpack('h', bytes(accRaw[4:6])))]
  #gyroMag = math.sqrt(gyroData[0]*gyroData[0]+gyroData[1]*gyroData[1]+gyroData[2]*gyroData[2])
  #gyroX = round(gyroData[0] * gyroGain, 2)
  #gyroY = round(gyroData[1] * gyroGain, 2)
  #gyroZ = round(gyroData[2] * gyroGain, 2)
  accMag = math.sqrt(accData[0]*accData[0]+accData[1]*accData[1]+accData[2]*accData[2])
  if accMag == 0:
    print('accMag was 0!')
    return [0,0,0]
  accXnorm = accData[0]/accMag
  accYnorm = accData[1]/accMag
  accZnorm = accData[2]/accMag
  accXangle = math.atan2(accYnorm, accZnorm)+3.1415
  accYangle = math.atan2(accZnorm, accXnorm)+3.1415 
  accZangle = math.atan2(accYnorm, accXnorm)+3.1415
  return [round(accXangle*180/3.1415,2), round(accYangle*180/3.1415,2), round(accZangle*180/3.1415,2)]
  #return [gyroX, gyroY, gyroZ, round(accXangle*180/3.1415,2), round(accYangle*180/3.1415,2), round(accZangle*180/3.1415,2)]
'''
I2C_ADDR = 0x13
def i2c(id, tick):
  global pi

  s, b, d = pi.bsc_i2c(I2C_ADDR)
  if b:
    if d[0] == ord('t'):
      print('sent={} FR={} recieved={} [{}]'.format(s>>16, s&0xfff,b,d))

      s, b, d = pi.bsc_i2c(I2C_ADDR,
                           "{}*".format(time.asctime()[11:13]))
'''

class GPSListener(threading.Thread):
  def __init__(self, gpsd):
    threading.Thread.__init__(self)
    self.gpsd = gpsd
    self.daemon = True

  def run(self):
    print('starting gps listener thread')

    while True:
      try:
        self.gpsd.next()
      except StopIteration:
        pass

#make sure to run 'sudo pigpiod' in the cmd before doing this

#multiply the gyroscope data by this to normalize it
gyroGain = 0.070

#now setup the accelerometer and gyroscope
bus = smbus.SMBus(1)
#accel address is 0x6a
#0x20 enables only the accelerometer, 0b01100111 is enable value
#0x10 enables the accelerometer and the gyroscope
#change numbers/add another statement to 
bus.write_byte_data(0x6a, 0x20, 0b01100111)
#0x21 is range reg, 0b00100000 is range value
bus.write_byte_data(0x6a, 0x21, 0b10000000)
#bus.write_byte_data(0x6a, 0x10, 0b01100000)

#sneaky thing so we start out being able to see the triceratops
accRaw = bus.read_i2c_block_data(0x6a, 0x28, 6)
rotOff = convertAcc(accRaw)
'''
for i in range(1000):
  #now actually get the data
  accRaw = bus.read_i2c_block_data(0x6a, 0x28, 6)
  if accRaw is not None:
    print(str(convertAcc(accRaw)))
    time.sleep(0.2)
'''

#also run 'sudo pigpiod'
#run 'sudo gpsd /dev/ttyS0 -F /var/run/gpsd.sock'
#setup the gps and the thread
threadLock = threading.Lock()
gpsd = gps.GPS(mode=WATCH_ENABLE)

while isnan(gpsd.fix.time):
  gpsd.next()
gpsOrigin = (gpsd.fix.latitude, gpsd.fix.longitude)
gpsThread = GPSListener(gpsd)
gpsThread.start()
#to check the gps lat and lon, just access gpsd.fix.latitude or longitude

isLeft = False

#do projection things
screenLL = (-18, -10, 80)
screenLR = (18, -10, 80)
screenUL = (-18, 10, 80)
eye = (-2 if isLeft else 2,0,0)
projMat = asymFrust(screenLL, screenLR, screenUL, eye, 1.0, 3000.0);
print("calculated asymmetric frustum projection matrix")
print(projMat)


#how to convert two gps coordinates to relative offsets
earthR = 637100000.0
#list of tuples (lat, lon)
gpsCoords = []
lastLine = None
#read the file
with open("GPSTest.txt") as f:
  a = f.readlines()
  print(len(a))
  for l in a:
    if lastLine == None or lastLine != l:
      lastLine = l
      gpsCoords += [[float(s) for s in l.strip().split(', ')]]
#print(gpsCoords)
print(len(gpsCoords))


#makes sure we can see triceratops, set to gpsOrigin for reality
startGPS = gpsCoords[0]

pathCoords = [(0,0,0)]
pathDists = []
pathDirs = [(1, 0, 0)]
for i in range(1, len(gpsCoords)):
  #zOff = (gpsCoords[i][0] - startGPS[0])/100.0*3.1415*earthR / 180.0;
  #xOff = (gpsCoords[i][1] - startGPS[1])*3.1415*earthR*math.cos((startGPS[0] + gpsCoords[i][0]) * 3.1415 / 180.0 / 100)/180.0/10.0;
  tempCoord = coordDif(gpsCoords[i], startGPS)
  if tempCoord[0] != pathCoords[-1][0] and tempCoord[2] != pathCoords[-1][2]:
    pathCoords += [[*tempCoord]]

#smoothing algorithm loading
for p in range(1, len(pathCoords)):
  pathDists += [math.sqrt((pathCoords[p][0]-pathCoords[p-1][0])*(pathCoords[p][0]-pathCoords[p-1][0])
                          +(pathCoords[p][0]-pathCoords[p-1][0])*(pathCoords[p][0]-pathCoords[p-1][0]))]
  pathDirs += [normalize((pathCoords[p][0]-pathCoords[p-1][0],
                          pathCoords[p][1]-pathCoords[p-1][1],
                          pathCoords[p][2]-pathCoords[p-1][2]))]
pathDirs += [pathDirs[-1]]

#print(pathCoords)
#tempZoff = -(rawGPSlat[i] - rawGPSlat[i - 1])/100.0*3.1415*earthR / 180.0;
#tempXoff = (rawGPSlon[i] - rawGPSlon[i - 1])*3.1415*earthR*cos((rawGPSlat[i] + rawGPSlat[i - 1]) * 3.1415 / 180.0 / 100)/180.0/10.0;

# Setup display and initialise pi3d
DISPLAY = pi3d.Display.create(x=0, y=0, w=0, h=0,
                         background = (0.2, 0.4, 0.6, 1), frames_per_second=0)
cam = pi3d.Camera()
cam.projection = projMat
camRotSpeed = 45.0

#these are the euler angles you want to modify
camRot = [0,0,0]

camDir = [0,0,1]
camUp = [0,1,0]
camPos = [0,0,0]
#this is the translation matrix for the actial positon
camPosMat = numpy.identity(4, dtype='float32')
print("display and camera set up")

shader = pi3d.Shader('uv_light')
#========================================

# load model_loadmodel
#change file_string to path for triceratops
#also change sx, sy, sz to 0.001 for triceratops
mymodel = pi3d.Model(file_string='trex.obj',
                name='Triceratops', x=0, y=-1, z=40,
                sx=1, sy=1, sz=1)
print("loaded triceratops");
mymodel.set_shader(shader)

# Fetch key presses
mykeys = pi3d.Keyboard()

#setup for path-following
chaseSpeed = 100
curDist = 0
#maximum value should be length - 1 (len of gpsCoords - 2)
curSeg = 0
curDir = pathDirs[0]

prevTime = time.time()

while 1:
  curTime = time.time()
  deltaT = curTime - prevTime
  prevTime = curTime

  if curSeg < len(pathDists):
    curDist += chaseSpeed*deltaT
    if curDist >= pathDists[curSeg]:
      curDist -= pathDists[curSeg]
      curSeg += 1

  if curSeg < len(pathDirs)-2:
    curDir = normalize(lerpV(pathDirs[curSeg], pathDirs[curSeg+1], curDist/pathDists[curSeg]))
  
  DISPLAY.clear()

  #set camera position with accelerometer
  #accData = (**MAGIC**)
  #camPos = [cam.eye[0]+accData[0]*deltaT*deltaT/2,
   #              cam.eye[1]+accData[1]*deltaT*deltaT/2,
    #             cam.eye[2]+accData[2]*deltaT*deltaT/2]

  #mymodel.xyz = 2*math.cos(curTime), 2*math.sin(curTime), 40
  #set camera rotation with the accelerometer
  #read accelerometer data
  accRaw = bus.read_i2c_block_data(0x6a, 0x28, 6)
  accData = convertAcc(accRaw)
  
  camRot[0] = accData[0]-rotOff[0]
  camRot[1] = accData[1]-rotOff[1]
  
  camDir = [-math.sin(camRot[1]*3.1415/180.0)*math.cos(camRot[0]*3.1415/180.0),
            math.sin(camRot[0]*3.1415/180.0),
            math.cos(camRot[0]*3.1415/180.0)*math.cos(camRot[1]*3.1415/180.0)]
  camUp = [-math.sin(camRot[1]*3.1415/180.0)*math.sin(camRot[0]*3.1415/180.0),
            math.cos(camRot[0]*3.1415/180.0),
            -math.sin(camRot[0]*3.1415/180.0)*math.cos(camRot[1]*3.1415/180.0)]
  camUp = vecRotVec(camUp, camDir, camRot[2]*3.1415/180.0)
  #camPos = [0,10*math.sin(curTime),0]

  #set the camera position based on the gps
  threadLock.acquire()
  camPos = [*coordDif((gpsd.fix.latitude, gpsd.fix.longitude), gpsOrigin)]
  threadLock.release()
  
  camPosMat[3,:3] = -numpy.array(camPos)
  cam.view = numpy.dot(camPosMat, LookAtMatrix(camDir,[0,0,0], camUp))
  cam.mtrx = numpy.dot(cam.view, cam.projection)

  #set the position of the model according to the smoothing algorithm
  #remove scaling factor later
  if curSeg < len(pathDirs) - 1:
    mymodel.xyz = (mymodel.xyz[0]+chaseSpeed*deltaT*curDir[0]/50,
                 mymodel.xyz[1],
                 mymodel.xyz[2]+chaseSpeed*deltaT*curDir[2]/50)
  #mymodel.rotateIncZ(0.01)
  #mymodel.rotateIncX(-0.0317543)
  #mymodel.rotateIncY(0.51)
  #mymodel.rotateToX(0.0);
  mymodel.rotateToY(-math.acos(dot((-1,0,0),curDir))*180/3.1415 if curDir[2] < 0 else math.acos(dot((-1,0,0),curDir))*180/3.1415)
  mymodel.draw()


  k = mykeys.read()
  if k >-1:
    if k==27:
      mykeys.close()
      DISPLAY.stop()
      break
    elif k==97:
      #cam.rotateY(camRotSpeed*deltaT)
      camRot[1] += camRotSpeed*deltaT
    elif k==100:
      #cam.rotateY(-camRotSpeed*deltaT)
      camRot[1] -= camRotSpeed*deltaT
    elif k==119:
      camRot[0] += camRotSpeed*deltaT
    elif k==115:
      camRot[0] -= camRotSpeed*deltaT
    elif k==101:
      camRot[2] += camRotSpeed*deltaT
    elif k==113:
      camRot[2] -= camRotSpeed*deltaT
    else:
      print(k)

  DISPLAY.swap_buffers()
DISPLAY.destroy()

