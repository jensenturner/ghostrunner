#!/usr/bin/python
#from __future__ import absolute_import, division, print_function, unicode_literals

""" large panda3d egg file with detailed texture
TODO it would be nice to show a comparison with a low poly version of this
with normal mapping for the details of the model.
"""
#import demo
import math
print("imported math")
from time import time
print("imported time")
import numpy
print("imported numpy")
import pi3d
print("imported pi3d")

#some random vector utility functions for calculating matrices
def dot(a, b):
  return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def addV(a, b):
  return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
def subV(a, b):
  return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def mag(a):
  return math.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])
def normalize(a):
  temp = mag(a)
  return (a[0]/temp, a[1]/temp, a[2]/temp)
def cross(a, b):
  return (a[1]*b[2] - a[2]*b[1], -a[0]*b[2]+a[2]*b[0], a[0]*b[1]-a[1]*b[0])

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

screenLL = (-18, -10, 80)
screenLR = (18, -10, 80)
screenUL = (-18, 10, 80)
eye = (0,0,0)
projMat = asymFrust(screenLL, screenLR, screenUL, eye, 1.0, 3000.0);
print("calculated asymmetric frustum projection matrix")
print(projMat)

# Setup display and initialise pi3d
DISPLAY = pi3d.Display.create(x=0, y=0, w=0, h=0,
                         background = (0.2, 0.4, 0.6, 1), frames_per_second=30)
cam = pi3d.Camera()
cam.projection = projMat
print("display and camera set up")

shader = pi3d.Shader('uv_light')
#========================================

# load model_loadmodel
mymodel = pi3d.Model(file_string='Triceratops.egg',
                name='Triceratops', x=0, y=-1, z=40,
                sx=0.001, sy=0.001, sz=0.001)
print("loaded triceratops");
mymodel.set_shader(shader)

# Fetch key presses
mykeys = pi3d.Keyboard()

prevTime = time()

while 1:
  curTime = time()
  deltaT = curTime - prevTime
  prevTime = curTime
  
  DISPLAY.clear()

  mymodel.xyz = 2*math.cos(curTime), 2*math.sin(curTime), 40
  mymodel.rotateIncZ(0.001)
  mymodel.rotateIncX(-0.00317543)
  mymodel.rotateIncY(0.11)
  #mymodel.rotateToX(0.0);
  mymodel.draw()


  k = mykeys.read()
  if k >-1:
    if k==27:
      mykeys.close()
      DISPLAY.stop()
      break
    else:
      print(k)

  DISPLAY.swap_buffers()
DISPLAY.destroy()

