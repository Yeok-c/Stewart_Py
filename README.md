# Stewart_Py
 Python implementation of stewart platform
 Usage: 
 
 
 Create object by defining parameters for the stewart platform
 
 Call function calculate([translation vector], [rotation vector]) to solve inverse kinematic for rotation angles for 6 servos
 
 [<img src="/doc/readme_resources/ezgif-7-487de93db9.gif" width="40%" height="40%">](/doc/readme_resources/ezgif-7-487de93db9.gif)

#### About the author

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

#### Future works
- [x] Barebone Python Implementation
- [ ] Jacobian Controller (Open loop)
- [ ] Error plots
- [ ] Simulate to find working envelope?
