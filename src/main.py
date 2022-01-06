import numpy as np
from stewart_controller import Stewart_Platform
import matplotlib.pyplot as plt



import sys    
def main():
    # Call object
    platform = Stewart_Platform(6.2, 5, 5.08, 10, 0.2269, 0.2269)

    # Initialize Plots
    fig, ax = plt.subplots()    

    # Loop through various angles
    for ix in range(-20, 20):
        angle = np.pi*ix/180
        servo_angles = platform.calculate_matrix( np.array([2,1,0]), np.array([0, angle, 0]) )
        # servo_angles = platform.calculate( np.array([0,0,0]), np.array([0, angle, 0]) )
        print(servo_angles)
        ax = platform.plot_platform()
        plt.pause(1000000000)

        plt.draw()

    
if __name__ == "__main__":
    main()


