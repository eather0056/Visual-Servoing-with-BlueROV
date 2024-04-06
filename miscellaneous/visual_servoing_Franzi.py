#!/usr/bin/env python
import numpy as np
    

        
#TODO Add others interactionMatrices

# the interactionMatrix for a point with rho theta representation
def interactionMatrixFeaturePointRhoTheta(rho,theta):
    Z=1
    Lrho = np.array([-np.cos(theta)/Z, -np.sin(theta)/Z, rho/theta, (1+rho**2)/np.sin(theta), -(1+rho**2)*np.cos(theta), 0])
    Ltheta = np.array([p.sin(theta)/(rho*Z), -np.cos(theta)/(rho*Z), 0, np.cos(theta)/rho, np.sin(theta)/rho, -1])
    L = np.stack((Lrho.T,Ltheta.T),axis=0)
    return L

#the stack of interactionMatrices for list of points with rho theta representation
def interactionMatrixFeaturePointRhoThetaList(points):
    n = int(np.shape(points)[0]/2)
    L = [[]];
    # for all the points
    point_reshaped = (np.array(points).reshape(n,2))
    for p in point_reshaped:
        Lp = interactionMatrixFeaturePointRhoTheta(p[0],p[1])
        if(iter == 0) : 
            L = Lp
        else: 
            L = np.concatenate((L,Lp))
    return L

# the interactionMatrix for a segment
def interactionMatrixFeaturePointSegment(xm,ym,l,alpha):
    rho = np.sqrt(xm**2+ym**2)
    theta = np.arctan(ym/xm)
    Lpoint = interactionMatrixFeaturePointRhoTheta(rho,theta)
    Lsegment = interactionMatrixFeaturePointRhoTheta(l,alpha)
    L = np.concatenate((Lpoint,Lsegment))
    return L
    

# the interactionMatrix for point coordinates
def interactionMatrixFeaturePoint2D(x,y,Z=1):
    Lx = np.array([ -1 / Z, 0, x / Z, x * y,-(1 + x * x), y])
    Ly = np.array([0, -1 / Z, y / Z, 1 + y * y, -x * y, -x ])
    L = np.stack((Lx.T,Ly.T),axis=0)
    return L


#the stack of interactionMatrices for list of points
def interactionMatrixFeaturePoint2DList(points):
    n = int(np.shape(points)[0]/2)
    Zs = np.ones(n)
        
    iter = 0 
    L = [[]];
    # for all the points
    point_reshaped = (np.array(points).reshape(n,2))
    for p in point_reshaped:
        Lp = interactionMatrixFeaturePoint2D(p[0],p[1],Zs[iter])
        if(iter == 0) : 
            L = Lp
        else: 
            L = np.concatenate((L,Lp))
        iter += 1
    return L



