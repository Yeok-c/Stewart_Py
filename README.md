# Stewart_Py

 [<img src="/doc/readme_resources/ezgif-7-487de93db9.gif" width="40%" height="40%">](/doc/readme_resources/ezgif-7-487de93db9.gif)

 Python implementation and step-by-step inverse kinematic explanation of stewart platform
 
[<img src="/doc/readme_resources/tutorial_ss.png">](/doc/readme_resources/tutorial_ss)

 
 Usage: 
 Fork this repo and copy the src folder into your project folder. Import package in python
 
     from src.stewart_controller import Stewart_Platform
 
 Create object by defining parameters for the stewart platform
      
     Stewart_Platform(r_B, r_P, lhl, ldl, Psi_B, Psi_P)
 
 Where   
 r_B = Radius of Base (Bottom)  
 r_P = Radius of Platform (Top)  
 lhl = Servo Horn Length  
 ldl = Rod length  
 Psi_B = Half of angle between two anchors on the base  
 Psi_P = Half of angle between two anchors on the platform  
 For details please refer to my tutorial https://github.com/Yeok-c/Stewart_Py  
    
 Call function calculate([translation vector], [rotation vector]) to solve inverse kinematic for rotation angles (Radian) for 6 servos
 
     servo_angles = platform.calculate( np.array([X,Y,Z), np.array([Pitch, Roll, Yaw]) )

 
#### Sources and additional reading
Robert Eisele's Explanation and js implementation: 
https://www.xarg.org/paper/inverse-kinematics-of-a-stewart-platform/
https://github.com/infusion/Stewart

hbartle's MATLAB implementation
https://github.com/hbartle/Stewart_Platform/

Others resources
https://github.com/NicHub/stewart-platform-esp32
https://github.com/daniel-s-ingram/stewart
https://github.com/felixros2401/Stewart-Platform

