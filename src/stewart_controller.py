import numpy as np
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt

class Stewart_Platform(object):   
    """
    Yeok 2022
    Stewart Platform Python Implementation
    Uses 6 Rotational Servos

    To initialize, pass 6 parameters
    r_B = Radius for circumscribed circle where all the anchor points for servo shaft lie on
    r_P = Radius for circumscribed circle where all anchor points for platform lie on
    lhl = |h| = length of servo horn
    ldl = |d| = length of rod
    alpha_B = 
    alpha_P = 
    """
    def __init__(s, r_B, r_P, lhl, ldl, alpha_B, alpha_P):
        pi = np.pi

        ## Define the Geometry of the platform

        # Beta (Angle)
        # Angle between the plane in which the servo arm moves and the xz-plane of the base CS.
        beta = np.array([ 
            pi+pi/2,        
            pi/2,
            2*pi/3+pi+pi/2, 
            2*pi/3+pi/2,
            4*pi/3+pi+pi/2, 
            4*pi/3+pi/2] )

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
            [ np.cos(theta_B[0]), -np.sin(theta_B[0]), 0],
            [ np.cos(theta_B[1]),  np.sin(theta_B[1]), 0],
            [-np.cos(theta_B[2]),  np.sin(theta_B[2]), 0],
            [-np.cos(theta_B[3]),  np.sin(theta_B[3]), 0],
            [-np.cos(theta_B[4]), -np.sin(theta_B[4]), 0],
            [-np.cos(theta_B[5]), -np.sin(theta_B[5]), 0] ])
        B = np.transpose(B)
            
        # Coordinates of the points where the rods 
        # are attached to the platform.
        P = r_P * np.array([ 
            [ np.cos(theta_P[0]), -np.sin(theta_P[0]), 0],
            [ np.cos(theta_P[1]),  np.sin(theta_P[1]), 0],
            [ np.cos(theta_P[2]),  np.sin(theta_P[2]), 0],
            [-np.cos(theta_P[3]),  np.sin(theta_P[3]), 0],
            [-np.cos(theta_P[4]), -np.sin(theta_P[4]), 0],
            [ np.cos(theta_P[5]), -np.sin(theta_P[5]), 0] ])
        P = np.transpose(P)

        # Save initialized variables
        s.r_B = r_B
        s.r_P = r_P
        s.lhl = lhl
        s.ldl = ldl
        s.alpha_B = alpha_B
        s.alpha_P = alpha_P

        # Calculated params
        s.beta = beta
        s.theta_B = theta_B
        s.theta_P = theta_P
        s.B = B
        s.P = P

        # Definition of the platform home position.
        z = np.sqrt( s.ldl**2 + s.lhl**2 - (s.P[0] - s.B[0])**2 - (s.P[1] - s.B[1])**2)
        s. home_pos= np.array([0, 0, z[0] ])
        # s.home_pos = np.transpose(home_pos)

        # Allocate for variables
        s.leg = np.zeros((3,6))
        s.llegl = np.zeros((6))
        s.angles = np.zeros((6))
        s.joint_B = np.zeros((3,6)) 

    def calculate_matrix(s, trans, orient):
        trans = np.transpose(trans)
        orient = np.transpose(orient)

        # Get rotation matrix of platform. RotZ* RotY * RotX -> matmul
        R = np.matmul( np.matmul(s.rotZ(orient[2]), s.rotY(orient[1])), s.rotX(orient[0]) )
        
        # Get leg length for each leg
        s.leg = np.repeat(trans[:, np.newaxis], 6, axis=1) + np.repeat(s.home_pos[:, np.newaxis], 6, axis=1) + np.matmul(np.transpose(R), s.P) - s.B 
        s.llegl = np.linalg.norm(s.leg, axis=0)

        # Position of legs, wrt to their individual bases
        lx = s.leg[0, :]
        ly = s.leg[1, :]
        lz = s.leg[2, :]

        # Calculate auxiliary quatities g, f and e
        g = s.llegl**2 - ( s.ldl**2 - s.lhl**2 )
        e = 2 * s.lhl * lz

        # Calculate servo angles for each leg
        for k in range(6):
            fk = 2 * s.lhl * (np.cos(s.beta[k]) * lx[k] + np.sin(s.beta[k]) * ly[k])
            
            # The wanted position could be achieved if the solution of this
            # equation is real for all i
            s.angles[k] = np.arcsin(g[k] / np.sqrt(e[k]**2 + fk**2)) - np.arctan2(fk,e[k])
            
            # Get postion of the point where a spherical joint connects servo arm and rod.
            s.joint_B[:, k] = np.transpose([ s.lhl * np.cos(s.angles[k]) * np.cos(s.beta[k]) + s.B[0,k],
                            s.lhl * np.cos(s.angles[k]) * np.sin(s.beta[k]) + s.B[1,k],
                            s.lhl * np.sin(s.angles[k]) ])
        
        # Position of leg in global frame
        s.leg = s.leg + s.B

        return s.angles
        
    def calculate(s, trans, orient):
        trans = np.transpose(trans)
        orient = np.transpose(orient)

        # Get rotation matrix of platform. RotZ* RotY * RotX -> matmul
        R = np.matmul( np.matmul(s.rotZ(orient[2]), s.rotY(orient[1])), s.rotX(orient[0]) )
        
        # Get leg length for each leg
        for i in range(6):        
            s.leg[:,i] = trans + s.home_pos + np.matmul(np.transpose(R), s.P[:,i]) - s.B[:,i]      
            s.llegl[i] = np.linalg.norm(s.leg[:,i])

        # Calculate auxiliary quatities g, f and e
        lx = s.leg[0, :]
        ly = s.leg[1, :]
        lz = s.leg[2, :]

        g = s.llegl**2 - ( s.ldl**2 - s.lhl**2 )
        e = 2 * s.lhl * lz

        # Calculate servo angles for each leg
        for k in range(6):
            fk = 2 * s.lhl * (np.cos(s.beta[k]) * lx[k] + np.sin(s.beta[k]) * ly[k])
            
            # The wanted position could be achieved if the solution of this
            # equation is real for all i
            s.angles[k] = np.arcsin(g[k] / np.sqrt(e[k]**2 + fk**2)) - np.arctan2(fk,e[k])
            
            # Get postion of the point where a spherical joint connects servo arm and rod.
            s.joint_B[:, k] = np.transpose([ s.lhl * np.cos(s.angles[k]) * np.cos(s.beta[k]) + s.B[0,k],
                            s.lhl * np.cos(s.angles[k]) * np.sin(s.beta[k]) + s.B[1,k],
                            s.lhl * np.sin(s.angles[k]) ])
        
        # Set params for class
        s.leg = s.leg + s.B

        return s.angles

    def plot_platform(s):
        ax = plt.axes(projection='3d') # Data for a three-dimensional line
        ax.set_xlim3d(-10, 10)
        ax.set_ylim3d(-10, 10)
        ax.set_zlim3d(0, 20)

        ax.add_collection3d(Poly3DCollection([list(np.transpose(s.B))]), zs='z')
        face_color = [0, 1, 0] # alternative: matplotlib.colors.rgb2hex([0.5, 0.5, 1])
        base_plot = Poly3DCollection([list(np.transpose(s.leg))])
        ax.add_collection3d(base_plot, zs='z')
        base_plot.set_facecolor(face_color)

        s.plot3D_line(ax, s.B, s.joint_B, 'red')
        s.plot3D_line(ax, s.joint_B, s.leg, 'black')
        s.plot3D_line(ax, s.B, s.leg, 'yellow')
        return ax

    def plot3D_line(s, ax, vec_arr_origin, vec_arr_dest, color_):
        for i in range(6):
            ax.plot([vec_arr_origin[0, i] , vec_arr_dest[0, i]],
            [vec_arr_origin[1, i], vec_arr_dest[1, i]],
            [vec_arr_origin[2, i],vec_arr_dest[2, i]],
            color=color_)

    def rotX(s, phi):
        rotx = np.array([
            [1,     0    ,    0    ],
            [0,  np.cos(phi), np.sin(phi)],
            [0, -np.sin(phi), np.cos(phi)] ])
        return rotx

    def rotY(s, theta):    
        roty = np.array([
            [np.cos(theta), 0, -np.sin(theta) ],
            [0         , 1,     0       ],
            [np.sin(theta), 0,  np.cos(theta) ] ])   
        return roty
        
    def rotZ(s, psi):    
        rotz = np.array([
            [ np.cos(psi), np.sin(psi), 0 ],
            [-np.sin(psi), np.cos(psi), 0 ],
            [   0        ,     0      , 1 ] ])   
        return rotz