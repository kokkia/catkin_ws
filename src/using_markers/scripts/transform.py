## coding: UTF-8
# 回転の座標変換
import numpy as np
import math
# 2D case
def Rz2D(theta):
    mat = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    # print(mat)
    return mat


# 3D case
def Rx(theta):
    mat = np.array([[1,0,0],[0,math.cos(theta),-math.sin(theta)],[0,math.sin(theta),math.cos(theta)]])
    return mat

def Ry(theta):
    mat = np.array([[math.cos(theta),0,math.sin(theta)],[0,1,0],[-math.sin(theta),0,math.cos(theta)]])
    return mat

def Rz(theta):
    mat = np.array([[math.cos(theta),-math.sin(theta),0],[math.sin(theta),math.cos(theta),0],[0,0,1]])
    return mat

def rpy(r, p, y):
    r = np.dot(Ry(p),Rx(r))
    mat = np.dot(Rz(y),r)
    return mat

def quaternion2R(q):#(w,x,y,z)
    r = np.zeros((3,3))
    r[0,0]=q[0]**2+q[1]**2-q[2]**2-q[3]**2
    r[0,1]=2*(q[1]*q[2]-q[3]*q[0])    
    r[0,2]=2*(q[3]*q[1]+q[2]*q[0])    
    r[1,0]=2*(q[1]*q[2]+q[3]*q[0])    
    r[1,1]=q[0]**2+q[2]**2-q[3]**2-q[1]**2
    r[1,2]=2*(q[2]*q[3]-q[1]*q[0])    
    r[2,0]=2*(q[3]*q[1]-q[2]*q[0])    
    r[2,1]=2*(q[2]*q[3]+q[1]*q[0])    
    r[2,2]=q[0]**2+q[3]**2-q[1]**2-q[2]**2
    return r

def R2quaternion(r):
    q = []
    q[0] = math.sqrt(1+r[0,0]+r[1,1]+r[2,2])
    q[1] = (r[2,1]-r[1,2])/4/q[0]
    q[2] = (r[0,2]-r[2,0])/4/q[0]
    q[3] = (r[1,0]-r[0,1])/4/q[0]
    return q
    