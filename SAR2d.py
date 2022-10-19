import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import numpy as np
import math

import time

class SAR2d:
    def __init__(self, h, CSP, S, dN, dE) -> None:
        self.h = h
        self.CSP = CSP
        self.S = S
        self.dN = dN
        self.dE = dE

    def parallelSearch_path(self, h, CSP, S, dN, dE):
        px, py = CSP[0], CSP[1]
        i = 1
        path = [CSP]
        while py <= CSP[0] + dN:
            if i%2==0:
                py = py + S
            elif (i-1)%4==0:
                px = CSP[1] + dE - S/2
            else:
                px = CSP[1] + S/2
            if py > CSP[0] + dN:
                break
            path = np.vstack((path, np.array([px, py, h])))
            i += 1
        return path
    

    def plotPath(self, path):
        fig = plt.figure(figsize=(10,10))
        ax = plt.axes(projection='3d')
        plt.cla()
        #ax.plot(path[:,0],path[:,1], path[:,2])
        
        return ax
    
    def computePoses(self, path):
        start_pose = np.array(np.hstack((path[0],
                                         np.arctan2(path[1][1]-path[0][1],
                                                    path[1][0]-path[0][0]))))
        poses = np.array([start_pose])
        for idx, p in enumerate(path):
            if idx < len(path) -1:
                pose = np.array(np.hstack((p, np.arctan2(path[idx+1][1]-path[idx][1],
                                                  path[idx+1][0]-path[idx][0]))))
                poses = np.vstack((poses, pose))
        return poses
                
        
    def plotRobot(self, ax, pose):#, params):
    	# r = params.sensor_range_m
        r = 2 #m
        ax.plot([pose[0]-r*np.cos(pose[3]), pose[0]+r*np.cos(pose[3])],
   			 [pose[1]-r*np.sin(pose[3]), pose[1]+r*np.sin(pose[3])],
   			 [pose[2], pose[2]], '--', linewidth=1, color='b')
        ax.plot([pose[0]-r*np.cos(pose[3]+np.pi/2), pose[0]+r*np.cos(pose[3]+np.pi/2)],
   		     [pose[1]-r*np.sin(pose[3]+np.pi/2), pose[1]+r*np.sin(pose[3]+np.pi/2)],
   		     [pose[2], pose[2]], '--', linewidth=1, color='b')
        ax.scatter(pose[0], pose[1], pose[2], marker='^')
        ax.quiver(pose[0], pose[1], pose[2], np.cos(pose[3]), np.sin(pose[3]), 0.0, length=0.2, normalize=True)
        


CSP = np.array([0,0,1])
S = 2
dN = 10
dE = 10
SAR = SAR2d(1,CSP, S, dN, dE)
path = SAR.parallelSearch_path(1, CSP, S, dN, dE)
poses = SAR.computePoses(path)         
ax = SAR.plotPath(path)
for pose in poses:
    SAR.plotRobot(ax, pose)
