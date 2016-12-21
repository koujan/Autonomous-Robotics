# -*- coding: utf-8 -*-
"""
Created on Mon Apr 25 17:40:56 2016

@author: mohammad
"""
import numpy as np
import random
import math
from PIL import Image
def RRT(q_goal):
    im = Image.open("/home/mohammad/catkin_ws/src/ar_proj/world/test_environment.png")
    siz=im.size
    nav_map = np.asarray(im)
    s=nav_map.shape
    if len(s)>2 :
    	nav_map=nav_map[:,:,0]
    q_start=np.array([425 ,375]) # center of the map
    edges=[0,0]
    k=30000
    robot_size=10
    delta_q=20
    step=1
    p=0.5
    # Initialize the vertices variable with q_start
    vertices=np.vstack((q_start,q_start))
    ver_index=2
    # For k samples repeat
    for i in range(k) :
        r=random.random() 
        if(r<p): # With p probability use q_rand = q_goal
            q_rand=q_goal
        else:    # Otherwise generate q_rand in the dimensions of the map
            q_rand=np.array([random.random() *siz[0] ,random.random() *siz[1]])
            q_rand=[math.ceil(q_rand[0]),math.ceil(q_rand[1])]
   
        # Find q_near from q_rand in vertices
        dist=np.sqrt((vertices[:,0]-q_rand[0])**2+(vertices[:,1]-q_rand[1])**2)
        ind=dist.argmin()
        q_near=vertices[ind,:]
        # generating q_new at delta_q distance from q_near in the direction to q_rand.
        theta=math.atan2(q_rand[1]-q_near[1],q_rand[0]-q_near[0])
        f=np.array([ [math.cos(theta),-math.sin(theta), q_near[0] ], [math.sin(theta) ,math.cos(theta), q_near[1] ] ])
        q_new=np.round(np.dot(f,[delta_q,0,1]))
        #q_new=q_new.T
        # in case the q_new goes beyond the q_rand   
        a=np.linalg.norm([ q_new[0]-q_near[0],q_new[1]-q_near[1] ])
        b=np.linalg.norm([ q_rand[0]-q_near[0],q_rand[1]-q_near[1] ])
        if(a>b): 
            q_new=q_rand
        # If q_new belongs to free space
        if(nav_map[q_new[1],q_new[0]]!=0 and nav_map[q_new[1]+robot_size,q_new[0]]!=0 and nav_map[q_new[1]-robot_size,q_new[0]]!=0 and nav_map[q_new[1],q_new[0]+robot_size]!=0  and nav_map[q_new[1],q_new[0]-robot_size]!=0):
           #If the edge between q_near and q_new belongs to free space
            belong_to_obstacle=0
            count=np.floor(np.linalg.norm([q_new[0]-q_near[0],q_new[1]-q_near[1]])/step) # step size between each of the 10 points on the current edge
            for j in range(int(count)):
                theta=math.atan2(q_new[1]-q_near[1],q_new[0]-q_near[0])
                f_incremental=np.array([ [math.cos(theta), -math.sin(theta), q_near[0]], [math.sin(theta), math.cos(theta), q_near[1]] ])
                xy=np.round(np.dot(f_incremental,[j*step,0,1]))
                if(nav_map[xy[1],xy[0]]==0  or nav_map[xy[1]+10,xy[0]]==0 or nav_map[xy[1]-10,xy[0]]==0 or nav_map[xy[1],xy[0]+10]==0 or nav_map[xy[1],xy[0]-10]==0):
                    belong_to_obstacle=1;
                    break;

            if(belong_to_obstacle==0):
                # Add q_new in vertices
                vertices=np.vstack((vertices,q_new))
                # Add [index(q_new) index(q_near)] in edges
                edges=np.vstack((edges,[ver_index, ind]))
                ver_index=ver_index+1
                if(q_new[0]==q_goal[0] and q_new[1]==q_goal[1]  ):
                   # Fill path and stop RRT function
                    path=edges[-1,:]
                    c=edges[-1,1]
                    while(c!=0):
                        path=np.append(path,edges[c-1,1])
                        c=edges[c-1,1] # index strats from zero in python
                    set_of_goals=vertices[path[::-1],:]
                    #print set_of_goals
                    #for i in range(set_of_goals.shape[0]):
                    #    print nav_map[set_of_goals[i,0],set_of_goals[i,1]]
                    return set_of_goals

    print 'Not enough samples to reach the goal'
    path=edges[-1,:]
    c=edges[-1,1]  
    while(c!=0):
        path=np.append(path,edges[c-1,1])
        c=edges[c-1,1]
    set_of_goals=vertices[path[::-1],:]
    return set_of_goals
