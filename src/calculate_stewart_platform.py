import time
import numpy as np
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import matplotlib.pyplot as plt

def calculate_stewart_platform( fig, r_B, r_P, 
lhl, ldl, alpha_B, alpha_P, 
trans, orient):
    pi = np.pi

## Define the Geometry of the platform

# Beta (Angle)
# Angle between the plane in which the servo arm moves and the xz-plane of the base CS.
    beta = np.array([ pi+pi/2,        pi/2,
             2*pi/3+pi+pi/2, 2*pi/3+pi/2,
             4*pi/3+pi+pi/2, 4*pi/3+pi/2])

# Theta_B (Polar coordinates)
# Direction of the points where the servo arm is attached to the servo axis.
    theta_B = np.array([ 
        alpha_B, 
        alpha_B,
        pi/3 + alpha_B, 
        pi/3 - alpha_B, 
        pi/3 - alpha_B, 
        pi/3 + alpha_B])

# Theta_P (Polar coordinates)
# Direction of the points where the rod is attached to the platform.
    theta_P = np.array([ 
        pi/3 - alpha_P, 
        pi/3 - alpha_P, 
        pi/3 + alpha_P, 
        alpha_P,
        alpha_P, 
        pi/3 + alpha_P] )

# Coordinate of the points where servo arms 
# are attached to the corresponding servo axis.
    B = r_B * np.array( [ 
        [ cos(theta_B[0]), -sin(theta_B[0]), 0],
        [ cos(theta_B[1]),  sin(theta_B[1]), 0],
        [-cos(theta_B[2]),  sin(theta_B[2]), 0],
        [-cos(theta_B[3]),  sin(theta_B[3]), 0],
        [-cos(theta_B[4]), -sin(theta_B[4]), 0],
        [-cos(theta_B[5]), -sin(theta_B[5]), 0] ])
    B = np.transpose(B)
     
# Coordinates of the points where the rods 
# are attached to the platform.
    P = r_P * np.array([ 
        [ cos(theta_P[0]), -sin(theta_P[0]), 0],
        [ cos(theta_P[1]),  sin(theta_P[1]), 0],
        [ cos(theta_P[2]),  sin(theta_P[2]), 0],
        [-cos(theta_P[3]),  sin(theta_P[3]), 0],
        [-cos(theta_P[4]), -sin(theta_P[4]), 0],
        [ cos(theta_P[5]), -sin(theta_P[5]), 0] ])
    P = np.transpose(P)
    
# Definition of the platform home position. The home position will be the
# position in which the angle between servo arm and rod is 90°. Because of
# the symmetric arrangement of the servos, h0 must be the same for all six
# positions.
    z = np.sqrt( ldl**2 + lhl**2 - (P[0] - B[0])**2 - (P[1] - B[1])**2)
    home_pos= np.array([0, 0, z[0] ])
    home_pos = np.transpose(home_pos)

## Calculate the needed leg length

# Calculate the transformation matrix for a transform from platform to base
# system with the given euler angles. If errors occure, you have to define
# the transformation matrices around x-, y- and z-axis for yourself.
    # matmul first two first, then ans with 3rd
    R = np.matmul( np.matmul(rotZ(orient[2]), rotY(orient[1])), rotX(orient[0]) ); 

# Calculate the leg vector and leg length for the new position of the
# platform for each servo.

# leg(:,2) = [2; 4; 6] % i think
# leg = eg:
# -4.31043156454323	-4.31043156454323	5.96303433229551	-1.65260276775227	-1.65260276775227	5.96303433229551
# -4.40026319743769	4.40026319743769	-1.53008760954086	-5.93035080697854	5.93035080697854	1.53008760954086
# 20.4720816253185	20.4720816253185	20.3573658611453	19.9844056231645	19.9844056231645	20.3573658611453

    leg = np.zeros((3,6))
    llegl = np.zeros((6))
    angles = np.zeros((6))

    for i in range(6):        
        leg[:,i] = trans + home_pos + np.matmul(R, P[:,i]) - B[:,i]      
        llegl[i] = np.linalg.norm(leg[:,i])

# Calculate the new servo angles
# % Calculate auxiliary quatities g, f and e
    lx = leg[0, :]
    ly = leg[1, :]
    lz = leg[2, :]

    g = llegl**2 - ( ldl**2 - lhl**2 )
    e = 2 * lhl * lz
    joint_B = np.zeros((3,6)) 

    for k in range(6):
        fk = 2 * lhl * (cos(beta[k]) * lx[k] + sin(beta[k]) * ly[k])
        
        # The wanted position could be achieved if the solution of this
        # equation is real for all i
        angles[k] = asin(g[k] / np.sqrt(e[k]**2 + fk**2)) - atan2(fk,e[k])
        
        # Get postion of the point where a spherical joint connects servo arm and rod.
        joint_B[:, k] = np.transpose([ lhl * cos(angles[k]) * cos(beta[k]) + B[0,k],
                        lhl * cos(angles[k]) * sin(beta[k]) + B[1,k],
                        lhl * sin(angles[k]) ])

    # return angles

## Plot the stewart platform

    ax = plt.axes(projection='3d') # Data for a three-dimensional line
    ax.set_xlim3d(-10, 10)
    ax.set_ylim3d(-10, 10)
    ax.set_zlim3d(0, 20)

    leg = leg + B
    ax.add_collection3d(Poly3DCollection([list(np.transpose(B))]), zs='z')
    face_color = [0, 1, 0] # alternative: matplotlib.colors.rgb2hex([0.5, 0.5, 1])
    base_plot = Poly3DCollection([list(np.transpose(leg))])
    ax.add_collection3d(base_plot, zs='z')
    base_plot.set_facecolor(face_color)

    # plt.axis([ 
    #     [-r_B - lhl, r_B + lhl],
    #     [-r_B - lhl, r_B + lhl],
    #     [-lhl,       ldl + lhl] ])

    plot3D_line(ax, B, joint_B, 'red')
    plot3D_line(ax, joint_B, leg, 'black')
    plot3D_line(ax, B, leg, 'yellow')

    plt.draw()
    plt.pause(0.001)
    return angles

def cos(x):
    return np.cos(x)

def sin(x):
    return np.sin(x)

def atan2(y,x):
    return np.arctan2(y,x)

def asin(x):
    return np.arcsin(x)

def acos(x):
    return np.arccos(x)

def rotX(phi):
    rotx = np.array([
        [1,     0    ,    0    ],
        [0,  cos(phi), sin(phi)],
        [0, -sin(phi), cos(phi)] ])
    return rotx

def rotY(theta):    
    roty = np.array([
        [cos(theta), 0, -sin(theta) ],
        [0         , 1,     0       ],
        [sin(theta), 0,  cos(theta) ] ])   
    return roty
    
def rotZ(psi):    
    rotz = np.array([
        [ cos(psi), sin(psi), 0 ],
        [-sin(psi), cos(psi), 0 ],
        [   0     ,     0   , 1 ] ])   
    return rotz

def plot3D_line(ax, vec_arr_origin, vec_arr_dest, color_):
    for i in range(6):
        ax.plot([vec_arr_origin[0, i] , vec_arr_dest[0, i]],
        [vec_arr_origin[1, i], vec_arr_dest[1, i]],
        [vec_arr_origin[2, i],vec_arr_dest[2, i]],
        color=color_)
    
def main():
    pi = np.pi
    fig = plt.figure()
    for ix in range(-30, 30):
        x = calculate_stewart_platform(fig, 6.2, 5, 5.08, 10, 0.2269, 0.2269, np.array([0,0,0]), np.array([0, pi*ix/180, 0]) )

if __name__ == "__main__":
    main()

