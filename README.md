# Mechanical Design

For our robot we decided to go with a pantograph design which is a four arm system that is in the shape of a pentagram. This system is controlled by two stepper motors and connected at the end effector point. For the mechanical design we wanted to base it around the ability to be easily assembled and changed with little effort. To do this, we made a simple base that would hold the paper, electronics, and a raised platform on a hinge that the motors would sit on. For the arms, we wanted something light yet sturdy so we cut 4 arm linkages out of acrylic. For the joints of these arms, we wanted something to allow for fluid motion so we used standard bolts with nyloc nuts to allow for a snug but moveable connection. For the pen holder at the end of the arms, we used a brass sleeve bearing, which has a natural lubrication and a rubber grommet to make for a secure connection. To implement this design a 3D model was made using solidworks. From this 3D model made .DXF files and used to laser cut the arms from an acrylic sheet. The base was made from simple materials consisting of wood, screws, and metal hinges. For the half degree of degree of freedom a solenoid was connected to the motor platform and lifted via the hinges. There were some issues with this design which will be expanded upon in the problems/issues section.

![](https://lh4.googleusercontent.com/SZZwtGOUzJ89svsozgHssgPbRi7nlT1S6XvAE-b0U0VexGxbFGso_pJU1KH_etNbBYWl4JJFMhRj8b_f-H4KUeB8_L-L7BkW4ymdtEgCfB5RSFv4pUDk6wdQjN4T4153tBJNKbnBDcrmsnoDYA)

                                           Figure 1:  3D model of pantograph robot


# Functionality

The Functionality of the Pantograph is intended to draw basic shapes, with two arms conjoined together and moving in a synchronized motion where at the intersection of the two arms, the pen is attached and drawing along the motion, if the pen in the instance is not suppose to draw anything , or go to the proper destination with no drawing, a solenoid is used (see Mechanical Design for more detail) to raise the platform , lifting the pen off the platform (see Figure 2 for a visual representation on how the Pantograph is suppose to work).  

 ![](https://lh6.googleusercontent.com/5IUN5j3HsNZq3CBchQPqR_x0ffJUl6nKl-C_El6-Taz2FRC1IzNqZJ2E8p8LZazR3SLiMAvfQEeoK9KV8bVTJ3-ISmeqk5Y6KgsNIFdRjx6D_kN6ASdfAA0dxuWj6juL4NJOQCrRaUXe4CWfSA)

                                          Figure 2: Pantograph Drawing a Square


# Kinematics

Forward Kinematics 

To calculate the Kinematics of the Pantograph, the design of the Pantograph is considered. In Figure 3 given four arms , four lengths are given : L1, L2, L3 , L4 and four angles : 𝛳<sub>1</sub>, 𝛳<sub>2</sub>, 𝛳<sub>3</sub>, 𝛳<sub>4</sub>.  The kinematics of the pantograph can be calculated. In Figure 3, C<sub>n</sub> and S<sub>n</sub> represent cos(𝛳<sub>n</sub>) and sin(𝛳<sub>n</sub>) respectively. Since we have two arms, the final destination, (x,y) is made up of two coordinates : (x<sub>1</sub>,y<sub>1</sub>), (x<sub>2</sub>, y<sub>2</sub>), with trigonometry , (x,y ) is calculated ( See Figure 3),    

  
  
![](https://lh5.googleusercontent.com/OD7QqwLgrSir-B4vhipu25mOrz42akblBx-kf8pcjTjSMVYlEzTFja3DxIS-QtVepUK_i-EugXoPkKuTxMyWt2mfPBpST9fn0wh8TM4SIJEacUqxtPdpmtuXVfTJ13sGSkQsrGCKfVAQrz4d2A)

                                          Figure 3: Kinematic model of system and Calculations

  
  
  
  
  


Inverse Kinematics 

The Inverse Kinematics is necessary , since in the Pantograph, it is taking the x-y coordinates as 

inputs , since the user wants to input coordinates and the robot needs to calculate the necessary angles to get the desired position. In order to calculate the inverse kinematics of the equation.The four kinematic equations are set up in a matrix as  f(𝛉) :

![](https://lh6.googleusercontent.com/WqU5BG1O3s8H_73QThg3nNdshV7MCDD5vR3u_UFWheKz-s4yslBk7QsnrUzmfgA5JcOtOzXXdUshLgNvoT8dzY4uLMxFJAQSd4828Cg5o7ieuwLn0xgEPZJfZEF5Yz7kCfmpcb8lzvvsGDylqg)          

                                              Figure 4 - x, 𝛉, and  f(𝛉) set  as matrices.

 

Once set, the Jacobian matrix was found 

![](https://lh4.googleusercontent.com/AHgEgvQdC7uFic1_x_XTE_458JCfB_K4ojjtjEOmtfa8pB7dc_NNplQna0UAJMNInLXdgSLUqUfPEtKlC4k5zLHcynLcstrYD8FEVs_vUWgCsr95b7teWWodbNASWR6yZYytsKGmSPt267wdOw)

                                                   Figure 5 - Jacobian Matrix of f(𝛉)

Hence the coordinates , x is now a function of 𝛉 and since the Jacobian matrix is found, then it is possible to find the velocity kinematics  of the system  .

![](https://lh3.googleusercontent.com/BTIF6r1Qs3YaLnuSAVwzl1ZAvGVc0-6q21d9ax0ieHdx61o_A4nq71chdMTVwWnMRunwau8U7SBCw7q0uPtolIA399HT07wRCLZqL3hf841waTS3KNszII-oHUVzubwBJL9YmlQKDDiV0AHuGw)

                                             Figure 6 - Velocity Kinematics of the Pantograph

 

Due to the level of complexity of the kinematics ,the velocity kinematics is utilized to help find the  Newton-Raphson algorithm,  a simple root-finding algorithm to iteratively solve the inverse kinematics, until the forward kinematics lands on a desired value for x. To do this first  g() needs to be solved for (see Figure 7), once solved,  g is solved for then (Figure 8).

  


![](https://lh6.googleusercontent.com/jdyqvvVFjSAMHN5ljVxXpGXnPFSyWpiH0tV7aph4QApA-HRmZ-BkYVUeYKQFt3WAwUgmxCRTLNYjK4mv7CcV08G-Xvy-0_t_SB4OsffORJrOJ_BG6aSfXgnYWG4kMdy2ofLTjlhO7m9hDD6Nqg)

                                              Figure 7 - Finding g()

  


                              ![](https://lh3.googleusercontent.com/bjsTP8qKihrwYnZaUPyxITK00mxPbml9PcfUbme0KS4ZqAhc7Z6ozgLSFM7onp5DQhST1ObHFZREzZ9hjUr0wH0RMUA7fFk-YV32RsMnfT4_swSnOrJ0jfH76cabBLGyx-AwZJutjuENvpxrLQ)

                                              Figure 8 - Finding g 

  


This new equation ( Figure 7) is selected so when g() is 0.  You can find the roots with the Newton Rhapson (Figure 9). The Newton-Raphson algorithm is a single-step iterative procedure for finding the roots of differential equations. In vector form and applied to this problem, The Newton-Raphson step would follow:

![](https://lh5.googleusercontent.com/0ztgpcRolQUN3t2mlrtfe7obo5jwE9HL2fbwNemfjhQ8oSwAQIionAeFaEZlAyeUdLilurssdB-seY52PkfDxcHg11Kx3aSWjnFvA-gy-QEbxMItIphEy67ZuFb0MdeeE6jMe1sb2hvzp0tk_Q) 

                                              Figure 9 - Newton-Raphson algorithm

where  n is the step number or number of iterations through the algorithm. The algorithm must stop when the value of g() is sufficiently close to zero by a threshold.




# Software Implementation and Design


## Driver Design

![](https://lh6.googleusercontent.com/gkv1brDWKpeKMTiEMTYdkuXm-bTqPADQgG-ZVcT7NvH8EIC0OtA1FPxfWnxLRKTamqyS67sAspmueixet6AmqKJvRMHXbnct_p8_dYp7mRVv2-Un-DUa_QLUXjQYZ6kSih7c8B09llsIHOKqzw)

                                             Figure 10: Class diagram of the motor driver class

For the driver design the focus was on basic functionality that could be expanded upon if desired. The init class sets up the TMC2208 and TMC 4210 motor drivers to allow the motor to move. It sets up the necessary pins and SPI signals on the nucleo board as well as the clock, velocity, acceleration, movement mode, and so on. The set velocity, set acceleration, and acceleration calculations methods are used in the init class to set up the TMC drivers. The send and receive methods will take the datagram to be sent and then pull the chip select signal for a specific peripheral low and send the datagram through SPI. The set position method will take the target position datagram as a parameter. The target position is then passed into the function and bit masked,  so that the correct value is sent to the X position register of the TMC 4210. The read position method will read the data in the X actual register of the TMC 4210 and return the data read.


## Implementation of Kinematics

To solve the kinematics of the system for many different end effector positions a script was made that implements the kinematics described above. This script takes position values and puts them through three methods to output the desired motor angles. These methods are g which finds the difference between f(θ) and the destination, dg_dtheta which finds the jacobian, and NewtonRaphson which implements the iterative method described earlier. The Newton-Raphson function is passed a lambda function that allows the function to run continuously as if it was given theta values. Additionally a helper function, matmul, which multiplies a 4x1 matrix and a 4x4 matrix is used since there are limitations to micropython.


## Parsing Vector Images/interpolation

To allow the system to draw the desired shapes a vector drawing software was used and then that image was saved in a hpgl format. This format gives x and y coordinates along with a set of commands for controlling machines which are separated by semicolons (;). These commands include ones such as pen up (PU), select pen (SP), and pen down (PD). Pen down/up are the two main commands that were used for this system. These hpgl files were read as one large string and then were split at each semicolon to separate each command. Once that was done, each command was separated into a list which consisted of the command, a list of x positions, and a list of y positions.

Since our system is not perfect and the robot does not move smoothly from one point to the next, each point requires interpolation which is the process of adding more points between points to make smoother lines. Once all of the values are parsed they go through an interpolation script which adds the extra points and scales/off sets the positional values for this specific system. After that is done, all of the points go through the Newton-Raphson process to be changed to motor angles and written to a text file stored in the system flash to reduce the memory usage of the system. 


## Task Organization

To make this system work multiple elements had to work in tandem to make the robot work effectively. To do this a system called cotask was used which works as a way of scheduling tasks to make them seem to work simultaneously. This system works with 5 different tasks. The first and main task is read command. The read command task will read commands/motor angle values from a precompile text file and determine from the text which task it should execute next. If the command is pen up (PU), then the system will put a flag high indicating that the pen up task should begin. In the pen up task the system will turn on a solenoid, which in turn lifts the platform that the motors are on and then it will raise the read flag, so that the next command can be read. 

If the value read is pen down (PD), then it will raise a pen down flag and go to the pen down task. In this task, it will turn off the solenoid and then raise the flag to read another command. If the next command is a number then it will raise a flag to start the move arms task, and will then read the values for motor one and motor two and put them into shares. In the move arms task it will take the motor values from the shares and send the position to the motor driver to move the arms to the specified position. After the position is set the system will go to the update LCD task which will take those motor positions and send them to an LCD to give user feedback. Once this is done the read flag will be set high and the system will continue.

![](https://lh3.googleusercontent.com/tipjlnzUHJFhlXqnyHyUC-EXn_yIb8mB2N57DP8JR4rU3nIr1HiyslRu4JpO0-RiN_xfPNFBczMakIh9j2T-LLUgYGd87k3JDHYPXT14p74n88D5ysTiloDU8CIdT31EusfcEWUMdJj1N50WWg)

                                          Figure 11 - state diagram of task management


# Issues/Improvements to make


## Mechanical

Arm to motor connection:

One of the main issues encountered was the connection of the acrylic arms to the motor shaft. A good connection was not able to be made which resulted in the arm sometimes slipping and not moving correctly. To reduce this slipping, a wedge was used to make for a tighter connection. This was helpful but it is not intended to be a permanent solution. Due to this poor connection and the weight of the arms/hardware connected, the arms would sag down making them harder to move in a smooth and controlled fashion.

To fix this problem it would have been best to instead implement a gear reduction system and have the arms mesh with a gear to make a more secure connection. Also the material of the arms at the joints and end effector should be changed to a lighter material to reduce the drag. On top of this it would be better to implement a clamping mechanism to hold the pen instead of a wedging solution to allow for more fine tuning and flexibility.

Pen choice:

For this system we used a felt tip pen which would often bleed out onto the paper which did not make clean lines. The felt pen allowed for easy writing as it did not require much pressure but was not good since the plotter did not move very quickly.

A solution to this problem would be to use a smooth roller ball pen and slightly more pressure on the pen itself. This would result in cleaner but slightly lighter lines. The pen clamp holder solution mentioned above would work well with this type of pen since it would allow for the pressure to be tuned until a good balance between line thickness and drag was found.

Solenoid/lifting arms:

A large issue for this system was the weight of the motor platform and lifting it to create the half degree of freedom. The original solution was to use a solenoid to lift the platform holding the motors but due to the weight of the material used (wood) and the motors/arms the solenoid was not able to lift it reliably. The solenoid could hold the platform but it could not lift it. As a temporary fix a spring was used to reduce some of the downward force of the platform which helped to barely lift the pen but was not fully effective.

A potential solution to this issue would be to change the material used to make the platform and change the lifting mechanism. A scissor lift system using a dc motor would have been a much better choice as it would have allowed for precise control of the height of the platform; however, it would have made moving the platform up and down a little slower but the improved performance would outweigh this downside. As for the material, a 3d printed platform with a light material would allow for better strength and would have reduced the load on the solenoid. A potential problem with this solution however is that the material could have melted due to the heat of the motors. This potential problem could be mediated by using an insulating layer.


## Software

Memory issues:

One of the main issues with the software for this system was the limitations of the on board memory. Python uses a lot of memory whenever it makes a variable because of the built-in overhead for integers. On top of that list in python take a lot of space due to them automatically resizing when new elements are added. This would become an issue whenever an attempt to use high resolution DPI was made or if more interpolation points were added. To get around this issue, points generated were stored within a text file and then read when the arms were to be moved. This would improve the system slightly but memory issues would still occur even when storing values in the flash memory instead of RAM.

To improve upon this issue instead of processing and storing the data on the board I would process the vector image data on the main PC and then store the values found to a file and then store that file within the flash memory of the board. In addition, the memory issue could be avoided completely by storing the data on the computer and sending the values serially over UART to the microcontroller. This would slow down the system considerably but would completely circumvent the memory issues.  

Newton-Raphson:

Calculation of the motor angles using the Newton-Raphson method would sometimes result in an infinite loop because the error of the system would not get below the threshold value.

To fix this issue I would choose a different method to determine the angles of the system. This could result in a slightly less accurate system but would stop the system from getting stuck in an infinite loop. A possible solution would be to use the inverse kinematics of the system to find the angle values given an x and y position. A possible reference is this paper on another pantograph system <http://www.cim.mcgill.ca/~haptic/pub/GC-QW-VH-IROS-05.pdf>


# Conclusions

Overall this system in its current state gives lackluster results and due to the issue encountered did not produce the drawing desired. However, with the improvements mentioned above then the robot would most likely improve greatly over the results that we achieved. And could possibly result in very clean drawings.


# Pictures and Videos

<https://youtu.be/ho-xggbm21k>

Link 1: Video of the robot trying to draw a triangle

<https://youtu.be/eQrWwerRZpA>

Link 2: Video of LCD screen displaying motor position values

![](https://lh3.googleusercontent.com/3ZxTpul4ipU8FTBOoZqZFzrDJ_StwBFknkeT-aYDVRrFpxzBSY9PnqulAyxIqGucii9x6t8NHzvVFt6g9jyqGodkejt8_JH3nkoYnMgfvjcboJ-apC0bm8yMoXFQ0480FQxqHAXCndsIBUrR-w)

                                              Figure 12: Overhead view of Pantograph robot

![](https://lh6.googleusercontent.com/vqunGED6_XVOLW2GzFY2TiMk3zumfJiHRfkD8eHOkKTmSa0NRtze8xbB0s-nmaeZQWo15JhnMOyechNu9fneflCPHrYLdmPUc3BpPQtbX5LOYdmr2bVfysCIGCGPFMTp3iExqwm92hgb_2qm1A)

                                  Figure 13: Motor driver board under platform connected to power supply and stepper motors
  ![](https://lh4.googleusercontent.com/H1tBMMkU9hrhPaxzceLJa7FawflzK02mjZ8etG1ZEA97BvmrS-xddlbrwwoPycsVyCKBj8gmacg2xwmVF0Vzj8Yeg-rokCfPRYSHeBm-CHk73_V4zoh-YoA2dgY7kM8G5OTUqrs-45Y2ZnyRAA)

                          Figure 14: STM Nucleo board connected to stepper driver and bread board with circuitry for driving the solenoid.
