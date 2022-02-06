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
    def __init__(s, r_B, r_P, lhl, ldl, gamma_B, gamma_P, ref_rotation):
        pi = np.pi
        beta = np.array([ 
            pi/2 + pi,        
            pi/2,
            2*pi/3 + pi/2 + pi , 
            2*pi/3 + pi/2,
            4*pi/3 + pi/2 + pi , 
            4*pi/3 + pi/2] )

        # Psi_B (Polar coordinates)
        psi_B = np.array([ 
            -gamma_B, 
            gamma_B,
            2*pi/3 - gamma_B, 
            2*pi/3 + gamma_B, 
            2*pi/3 + 2*pi/3 - gamma_B, 
            2*pi/3 + 2*pi/3 + gamma_B])

        # psi_P (Polar coordinates)
        # Direction of the points where the rod is attached to the platform.
        psi_P = np.array([ 
            pi/3 + 2*pi/3 + 2*pi/3 + gamma_P,
            pi/3 + -gamma_P, 
            pi/3 + gamma_P,
            pi/3 + 2*pi/3 - gamma_P, 
            pi/3 + 2*pi/3 + gamma_P, 
            pi/3 + 2*pi/3 + 2*pi/3 - gamma_P])

        psi_B = psi_B + np.repeat(ref_rotation, 6)
        psi_P = psi_P + np.repeat(ref_rotation, 6)
        beta = beta + np.repeat(ref_rotation, 6)

        # Coordinate of the points where servo arms 
        # are attached to the corresponding servo axis.
        B = r_B * np.array( [ 
            [ np.cos(psi_B[0]), np.sin(psi_B[0]), 0],
            [ np.cos(psi_B[1]), np.sin(psi_B[1]), 0],
            [ np.cos(psi_B[2]), np.sin(psi_B[2]), 0],
            [ np.cos(psi_B[3]), np.sin(psi_B[3]), 0],
            [ np.cos(psi_B[4]), np.sin(psi_B[4]), 0],
            [ np.cos(psi_B[5]), np.sin(psi_B[5]), 0] ])
        B = np.transpose(B)
            
        # Coordinates of the points where the rods 
        # are attached to the platform.
        P = r_P * np.array([ 
            [ np.cos(psi_P[0]),  np.sin(psi_P[0]), 0],
            [ np.cos(psi_P[1]),  np.sin(psi_P[1]), 0],
            [ np.cos(psi_P[2]),  np.sin(psi_P[2]), 0],
            [ np.cos(psi_P[3]),  np.sin(psi_P[3]), 0],
            [ np.cos(psi_P[4]),  np.sin(psi_P[4]), 0],
            [ np.cos(psi_P[5]),  np.sin(psi_P[5]), 0] ])
        P = np.transpose(P)

        # Save initialized variables
        s.r_B = r_B
        s.r_P = r_P
        s.lhl = lhl
        s.ldl = ldl
        s.gamma_B = gamma_B
        s.gamma_P = gamma_P

        # Calculated params
        s.beta = beta
        s.psi_B = psi_B
        s.psi_P = psi_P
        s.B = B
        s.P = P

        # Definition of the platform home position.
        z = np.sqrt( s.ldl**2 + s.lhl**2 - (s.P[0] - s.B[0])**2 - (s.P[1] - s.B[1])**2)
        s. home_pos= np.array([0, 0, z[0] ])
        # s.home_pos = np.transpose(home_pos)

        # Allocate for variables
        s.l = np.zeros((3,6))
        s.lll = np.zeros((6))
        s.angles = np.zeros((6))
        s.H = np.zeros((3,6)) 

    def calculate(s, trans, rotation):
        trans = np.transpose(trans)
        rotation = np.transpose(rotation)

        # Get rotation matrix of platform. RotZ* RotY * RotX -> matmul
        R = np.matmul( np.matmul(s.rotZ(rotation[2]), s.rotY(rotation[1])), s.rotX(rotation[0]) )
        # R = np.matmul( np.matmul(s.rotX(rotation[0]), s.rotY(rotation[1])), s.rotZ(rotation[2]) )

        # Get leg length for each leg
        # leg = np.repeat(trans[:, np.newaxis], 6, axis=1) + np.repeat(home_pos[:, np.newaxis], 6, axis=1) + np.matmul(np.transpose(R), P) - B 

        # Get leg length for each leg
        s.l = np.repeat(trans[:, np.newaxis], 6, axis=1) + np.repeat(s.home_pos[:, np.newaxis], 6, axis=1) + np.matmul(R, s.P) - s.B 
        s.lll = np.linalg.norm(s.l, axis=0)

        # Position of leg in global frame
        s.L = s.l + s.B

        # Position of legs, wrt to their individual bases, split for clarity.
        lx = s.l[0, :]
        ly = s.l[1, :]
        lz = s.l[2, :]

        # Calculate auxiliary quatities g, f and e
        g = s.lll**2 - ( s.ldl**2 - s.lhl**2 )
        e = 2 * s.lhl * lz

        # Calculate servo angles for each leg
        for k in range(6):
            fk = 2 * s.lhl * (np.cos(s.beta[k]) * lx[k] + np.sin(s.beta[k]) * ly[k])
            
            # The wanted position could be achieved if the solution of this
            # equation is real for all i
            s.angles[k] = np.arcsin(g[k] / np.sqrt(e[k]**2 + fk**2)) - np.arctan2(fk,e[k])
            
            # Get postion of the point where a spherical joint connects servo arm and rod.
            s.H[:, k] = np.transpose([ s.lhl * np.cos(s.angles[k]) * np.cos(s.beta[k]) + s.B[0,k],
                            s.lhl * np.cos(s.angles[k]) * np.sin(s.beta[k]) + s.B[1,k],
                            s.lhl * np.sin(s.angles[k]) ])
        
        return s.angles

    def plot3D_line(s, ax, vec_arr_origin, vec_arr_dest, color_):
        for i in range(6):
            ax.plot([vec_arr_origin[0, i] , vec_arr_dest[0, i]],
            [vec_arr_origin[1, i], vec_arr_dest[1, i]],
            [vec_arr_origin[2, i],vec_arr_dest[2, i]],
            color=color_)

    def plot_platform(s):
        ax = plt.axes(projection='3d') # Data for a three-dimensional line
        ax.set_xlim3d(-100, 100)
        ax.set_ylim3d(-100, 100)
        ax.set_zlim3d(0, 200)
        ax.set_xlabel('x-axis')
        ax.set_ylabel('y-axis')
        ax.set_zlabel('z-axis')

        # ax.add_collection3d(Poly3DCollection([list(np.transpose(s.B))]), zs='z')
        ax.add_collection3d(Poly3DCollection([list(np.transpose(s.B))], facecolors='green', alpha=0.25))

        # ax.add_collection3d(base_plot, zs='z')
        ax.add_collection3d(Poly3DCollection([list(np.transpose(s.L))], facecolors='blue', alpha=0.25))

        s.plot3D_line(ax, s.B, s.H, 'red')
        s.plot3D_line(ax, s.H, s.L, 'black')
        s.plot3D_line(ax, s.B, s.L, 'orange')
        return ax

    def plot_platform_g(s, global_trans):
        ax = plt.axes(projection='3d') # Data for a three-dimensional line
        ax.set_xlim3d(-400, 400)
        ax.set_ylim3d(-400, 400)
        ax.set_zlim3d(0, 200)
        ax.set_xlabel('x-axis')
        ax.set_ylabel('y-axis')
        ax.set_zlabel('z-axis')

        ax.add_collection3d(Poly3DCollection([list(np.transpose(s.B))], facecolors='green', alpha=0.25))
        ax.add_collection3d(Poly3DCollection([list(np.transpose(s.L))], facecolors='blue', alpha=0.25))

        s.plot3D_line(ax, s.B, s.H, 'red')
        s.plot3D_line(ax, s.H, s.L, 'black')
        s.plot3D_line(ax, s.B, s.L, 'orange')
        return ax

    def rotX(s, phi):
        rotx = np.array([
            [1,     0    ,    0    ],
            [0,  np.cos(phi), -np.sin(phi)],
            [0,  np.sin(phi), np.cos(phi)] ])
        return rotx

    def rotY(s, theta):    
        roty = np.array([
            [np.cos(theta), 0, np.sin(theta) ],
            [0         , 1,     0       ],
            [-np.sin(theta), 0,  np.cos(theta) ] ])   
        return roty
        
    def rotZ(s, psi):    
        rotz = np.array([
            [ np.cos(psi), -np.sin(psi), 0 ],
            [np.sin(psi), np.cos(psi), 0 ],
            [   0        ,     0      , 1 ] ])   
        return rotz

    # Roll yaw pitch notation
    # def rotX(s, phi):
    #     rotx = np.array([
    #         [1,     0    ,    0    ],
    #         [0,  np.cos(phi), np.sin(phi)],
    #         [0, -np.sin(phi), np.cos(phi)] ])
    #     return rotx

    # def rotY(s, theta):    
    #     roty = np.array([
    #         [np.cos(theta), 0, -np.sin(theta) ],
    #         [0         , 1,     0       ],
    #         [np.sin(theta), 0,  np.cos(theta) ] ])   
    #     return roty
        
    # def rotZ(s, psi):    
    #     rotz = np.array([
    #         [ np.cos(psi), np.sin(psi), 0 ],
    #         [-np.sin(psi), np.cos(psi), 0 ],
    #         [   0        ,     0      , 1 ] ])   
    #     return rotz