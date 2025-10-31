import numpy as np
from math import atan, pi
#Arm Lengths
a1=4.5
a2=6
a3=5.4

   
def point_to_rad(p1, p2): # converts 2D cartesian points to polar angles in range 0 - 2pi
    #since atan has a range of -pi/2 to pi/2
        
    if (p1 > 0 and p2 >= 0): return atan(p2/(p1))
    elif (p1 == 0 and p2 >= 0): return pi/2
    elif (p1 < 0 and p2 >= 0): return -abs(atan(p2/p1)) + pi
    elif (p1 < 0 and p2 < 0): return atan(p2/p1) + pi
    elif (p1 > 0 and p2 < 0): return -abs(atan(p2/p1)) + 2*pi
    elif (p1 == 0 and p2 < 0): return pi * 3/2
    elif (p1 == 0 and p2 == 0): return pi * 3/2 # edge case

def Inverse_kinematics(x,y,z,leg):
 
    L = np.sqrt(x**2 + y**2 + z**2)

    if L > (a1 + a2 + a3):
        raise ValueError("Target is outside the workspace.")

    #Calculating T1
    A = np.sqrt(x**2 + y**2)
    
    
    if(leg == 0):
        print("Leg is right")
        T1 = point_to_rad(x,y) + np.arccos(a1/A)
        if T1>np.pi/2:
            T1=T1-np.pi*2
    elif(leg==1):
        print("Leg is left")
        T1 = point_to_rad(x,y) - np.arccos(a1/A)


    rot=np.matrix([[np.cos(-T1),-np.sin(-T1),0,0],
                    [np.sin(-T1),np.cos(-T1),0,0],
                    [0,0,1,0],
                    [0,0,0,1]])
    
    trans=np.matrix([[1,0,0,-a1*np.cos(T1)],
                     [0,1,0,-a1*np.sin(T1)],
                     [0,0,1,0],
                     [0,0,0,1]])
    
    actual_coordinate=np.matrix([[x],
                                 [y],
                                 [z],
                                 [1]])
 
    transformed_coordinate=np.dot(rot,np.dot(trans,actual_coordinate))

    xdash,ydash,zdash,_=transformed_coordinate[0][0].item(),transformed_coordinate[1][0].item(),transformed_coordinate[2][0].item(),transformed_coordinate[3][0]

    # CALCULATING T2
    B = np.sqrt(ydash**2+zdash**2)
    alpha1 = point_to_rad(ydash,zdash)
    beta1 = np.arccos((B**2 + a2**2 - a3**2)/(2*B*a2))
    T2 = alpha1 - beta1
    beta2 = np.arccos((a2**2 + a3**2 - B**2)/(2*a2*a3))
    T3 = np.pi - beta2

    # Convert to degree
    T1 = np.degrees(T1)
    T2 = np.degrees(T2)
    T3 = np.degrees(T3)

    print('T1 = ',T1)
    print('T2 = ',T2)
    print('T3 = ',T3)
    

while(True):
    leg = int(input("Which side leg? 0 for RIGHT, 1 for LEFT: "))
    x = float(input("X: "))
    y = float(input("Y: "))
    z = float(input("Z: "))
    Inverse_kinematics(x,y,z,leg)