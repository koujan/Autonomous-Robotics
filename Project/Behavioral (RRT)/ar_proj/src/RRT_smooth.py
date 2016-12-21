# -*- coding: utf-8 -*-
"""
Created on Mon May  9 05:44:41 2016

@author: mohammad
"""
import numpy as np
import random
import math
from PIL import Image
def smooth(vertices,delta):
    im = Image.open("/home/mohammad/catkin_ws/src/ar_proj/world/test_environment.png")
    siz=im.size
    nav_map = np.asarray(im)
    s=nav_map.shape
    if len(s)>2 :
        nav_map=nav_map[:,:,0]
    robot_size=10
    flip_path=range(vertices.shape[0])
    v_end=vertices[flip_path[-1],:]  # v_end initially equals the goal point
    v_end_index=flip_path[-1]
    path_smooth=np.array([flip_path[-1],flip_path[-1]])   #  path_smooth initially has the goal point index
    while (path_smooth[-1]!=0): # % loop for generating the smoothed path
        i=0
        while (i<v_end_index): # loop for finding a line of sight in the path
            v_start=vertices[flip_path[i],:]
            belong_to_obstacle=0
            ite=np.floor(np.linalg.norm([v_start[0]-v_end[0],v_start[1]-v_end[1]])/delta)
            for j in range(int(ite)): # incremental checking for the line connecting two nodes
                theta= math.atan2(v_end[1]-v_start[1],v_end[0]-v_start[0])             
                f=np.array([ [math.cos(theta), -math.sin(theta), v_start[0] ], [math.sin(theta) ,math.cos(theta), v_start[1] ] ])# transformation matrix
                xy=np.round(np.dot(f,[j*delta,0,1])) 
                  #xy=xy.T
                if(nav_map[xy[1],xy[0]]==0 or nav_map[xy[1]+robot_size,xy[0]]==0 or nav_map[xy[1]-robot_size,xy[0]]==0 or nav_map[xy[1],xy[0]+robot_size]==0  or nav_map[xy[1],xy[0]-robot_size]==0): # if the line pass through an obstacle
                    belong_to_obstacle=1
                    break
                  
           
            if(belong_to_obstacle==0): # the line lies entirely in the free space
                path_smooth=np.append(path_smooth, flip_path[i])
		print path_smooth
                break
            else:
                i=i+1
        #check the visibility of a nearer vertex to the start
        v_end_index=flip_path[i]
        v_end=vertices[flip_path[i],:]
    
    path_smooth=path_smooth[1::]
    path_smooth=path_smooth[::-1]	    
    return vertices[path_smooth,:]

