## Mechanical Design

Our robot design is a pantograph which is a four arm system that is in the shape of a pentagram which is controlled by two motors and connected at the end effector point. For the mechanical design we wanted something simple that could be easily assembled and changed with little effort. To do this, we made a simple base that would hold the paper, electronics, and the motors. For the arms, we wanted something light yet sturdy so we cut 4 arm linkages out of acrylic. For the joints of these arms, we wanted something to allow for fluid motion so we used standard bolts with nyloc nuts to allow for a snug but moveable connection. For the pen hold at the end of the arms, we used a brass sleeve bearing, which has a natural lubrication and a rubber grommet to make for a secure connection. To implement this design a 3D model was made using solidworks which was then used to create the parts needed. For the arms, the solidworks files were converted to .DXF files and used to laser cut an acrylic sheet. The base was made from simple materials consisting of wood, screws, and metal hinges.  There was some issues with this design which will be expanded upon in the problems/issues section



<p id="gdcalert1" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image1.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert2">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image1.png "image_tooltip")


Figure 1:  3D model of pantograph robot


## Functionality

	

The Functionality of the Pantograph is intended to draw basic shapes, with two arms conjoined together and moving in a synchronized motion where at the intersection of the two arms, the pen is attached and drawing along the motion , if the pen in the instance is not suppose to draw anything , or go to the proper destination with no drawing, a solenoid is used (see Mechanical Design for more detail) to raise the platform , lifting the pen off the ground (see Figure 1 for a visual representation on how the Pantograph is suppose to work).  

 

Figure 2: Pantograph Drawing a Square


## Kinematics

Forward Kinematics 

To calculate the Kinematics of the Pantograph, the design of the Pantograph is considered. In Figure 2 given four arms , four lengths are given : L1, L2, L3 , L4 and four angles : 𝛳<sub>1</sub>, 𝛳<sub>2</sub>, 𝛳<sub>3</sub>, 𝛳<sub>4</sub>.  The kinematics of the pantograph can be calculated. In Figure 2, C<sub>n</sub> and S<sub>n</sub> represent cos(𝛳<sub>n</sub>) and sin(𝛳<sub>n</sub>) respectively. Since we have two arms, the final destination, (x,y) is made up of two coordinates : (x<sub>1</sub>,y<sub>1</sub>), (x<sub>2</sub>, y<sub>2</sub>), with trigonometry , (x,y ) is calculated ( See Figure 2),    

Figure 2: Kinematic model of system and Calculations

Inverse Kinematics 

The Inverse Kinematics is necessary , since in the Pantograph, it is taking the x-y coordinates as 

inputs , since the user wants to input coordinates and the robot needs to calculate the necessary angles to get the desired position. In order to calculate the inverse kinematics of the equation.The four kinematic equations are set up in a matrix as  f(𝛉) :


                                        

<p id="gdcalert2" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image2.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert3">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image2.png "image_tooltip")




<p id="gdcalert3" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image3.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert4">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image3.png "image_tooltip")
          

<p id="gdcalert4" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image4.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert5">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image4.png "image_tooltip")
  

Figure 3 - x, 𝛉, and  f(𝛉) set  as matrices.

 

Once set, the Jacobian matrix was found 

Figure 4 - Jacobian Matrix of f(𝛉)

Hence the coordinates , x is now a function of 𝛉 and since the Jacobian matrix is found, the it is possible to find the velocity kinematics  of the system  .



<p id="gdcalert5" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image5.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert6">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image5.png "image_tooltip")


Figure 5 - Velocity Kinematics of the Pantograph

 

Due to the level of complexity of the kinematics ,the velocity kinematics is utilized to help find the  Newton-Raphson algorithm,  a simple root-finding algorithm to iteratively solve the inverse kinematics, until the forward kinematics lands on a desired value for x. To do this first  

<p id="gdcalert6" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: equation: use MathJax/LaTeX if your publishing platform supports it. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert7">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>

 needs to be solved for (see Figure 6), once solved,  

<p id="gdcalert7" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: equation: use MathJax/LaTeX if your publishing platform supports it. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert8">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>

 is solved for then.



<p id="gdcalert8" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image6.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert9">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image6.png "image_tooltip")


Figure 6 - Finding 

<p id="gdcalert9" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: equation: use MathJax/LaTeX if your publishing platform supports it. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert10">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>



                              

<p id="gdcalert10" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image7.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert11">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image7.png "image_tooltip")


Figure 7 - Finding 

<p id="gdcalert11" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: equation: use MathJax/LaTeX if your publishing platform supports it. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert12">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>

 

This new equation ( see figure 6) is selected so when 

<p id="gdcalert12" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: equation: use MathJax/LaTeX if your publishing platform supports it. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert13">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>

 is 0.  You can find the roots with the Newton Rhapson (Figure 8). The Newton-Raphson algorithm is a single-step iterative procedure for finding the roots of differential equations. In vector form and applied to this problem, The Newton-Raphson step would follow:



<p id="gdcalert13" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image8.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert14">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image8.png "image_tooltip")
 

Figure 8 - Newton-Raphson algorithm

where  n is the step number or number of iterations through the algorithm. The algorithm must stop when the value of 

<p id="gdcalert14" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: equation: use MathJax/LaTeX if your publishing platform supports it. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert15">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>

 is sufficiently close to zero by a threshold.


## Software Implementation and Design


### Driver Design

		

<p id="gdcalert15" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image9.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert16">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image9.png "image_tooltip")


Figure X: Class diagram of the motor driver class

	For the driver design we wanted to implement a simple driver class that would work with both motors and cover the basic functionality. The init class sets up the TMC2208 and TMC 4210 motor drivers to allow the motor to move. It sets up the necessary pins and SPI signals on the nucleo board as well as the clock, velocity, acceleration, movement mode, and so on. The set velocity, set acceleration, and acceleration calculations methods are used in the init class to set up the TMC chips. The send and receive methods will take the datagram to be sent and will pull the chip select signal for a specific peripheral low and send the datagram through SPI. The set position method will take the target position datagram as a parameter. The target position is then passed into the function and bit masked,  so that the correct value is sent to the X position register of the TMC 4210. The read position method will read the data in the X actual register of the TMC 4210 and return the data it read.


### Implementation of Kinematics

		To solve the kinematics of the system for many different end effector positions a script was made that implements the kinematics described above. This script takes position values and puts them through an iterative method that is called Newton-Raphson which will take a guess of theta values and then iteratively go though angle values based off the previous value until angles are that that results in a positional value that matches the input. This is accomplished with three main methods Newto-nRaphson, g, and dg_dtheta. The Newton-Raphson function is passed a lambda function that allows the function to run continuously as if it was given theta values. Methods g and dg_dtheta are used in computing the new theta values as f(θ) and the Jacobian of the system.


### Parsing Vector Images/interpolation

	To allow the system to draw the desired shapes a vector drawing software was used and then that image was saved in a hpgl format which gives x and y coordinates along with a set of commands for controlling machines which are separated by semicolons (;). These commands include ones such as pen up (PU), select pen (SP), and pen down (PD). Pen down/up are the two main commands that were used for this system. These hpgl files were read as one large string and then were split at each semicolon to separate each command. Once that was done then each command was separated into a list which consisted of the command, a list of x positions, and a list of y positions.

	

	Since our system is not perfect and the robot does not move smoothly from one point to the next, each point requires interpolation which is the process of adding more points between points to make smoother lines. Once all of the values are parsed they go through an interpolation script which adds the extra points and scales/off sets the positional values for this specific system. After that is done, all of the points go through the Newton-Raphson process to be changed to motor angles and written to a text file stored in the system flash to reduce the memory usage of the system. 


### Task Organization

	To make this system work multiple elements had to work in tandem to make the robot work effectively. To do this a system called cotask was used which works as a way of scheduling tasks to make them seem to work simultaneously. This system works with 5 different tasks. The first and main task is the read command. The read command task will read commands/motor angle values from a precompile text file and determine from the text which task it should execute next. If the command is pen up (PU), then the system will put a flag high indicating that the pen up task should begin. In the pen up task the system will turn on a solenoid, which in turn lifts the platform that the motors are on  and then it will raise the read flag, so that the next command can be read. 

If the value read is pen down (PD), then it will raise a pen down flag and go to the pen down task. In the pen down task, it will turn off the solenoid and then raise the flag to read another command. If the command is a number then it will raise a flag to start the move arms task, and it will read the values for motor one and motor two and put them into shares. In the move arms task it will take the motor values from the shares and send the position to the motor driver to move the arms to the specified position. After the position is set the system will go to the update LCD task which will take those motor positions and send them to an LCD to give user feedback. Once this is done the read flag will be set high and the system will continue.



<p id="gdcalert16" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image10.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert17">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image10.png "image_tooltip")


Figure 9- state diagram of task management


## Issues/Improvements to make


### Mechanical

Arm to motor connection:

	One of the main issues encountered was the connection of the acrylic arms to the motor shaft. A good connection was not able to be made which resulted in the arm sometimes slipping and not moving correctly. To reduce this slipping, a wedge was used to make for a tighter connection. This was  helpful but it is not intended to be a permanent solution. Due to this poor connection, the weight of the arms and hardware connected, the arms would sag down making them harder to move in a smooth and controlled fashion.

	To fix this problem it would have been best to instead implement a gear reduction system and have the arms mesh with a gear to make a more secure connection. Also the material of the arms at the joints and end effector should be changed to a lighter material to reduce the drag. On top of this it would be better to implement a clamping mechanism to hold the pen instead of a wedging solution to allow for more fine tuning and flexibility.

		

Pen choice:

	For this system we used a felt tip pen which would often bleed out onto the paper which did not make clean lines. The felt pen allowed for easy writing as it did not require much pressure but was not good since the plotter did not move very quickly.

	A solution to this problem would be to use a smooth roller ball pen and slightly more pressure on the pen itself. This would result in cleaner but slightly lighter lines. The pen clamp holder solution mentioned above would work well with this type of pen since it would allow for the pressure to be tuned until a good balance between line thickness and drag was found.

		

Solenoid/lifting arms:

	A large issue for this system was the weight of the motor platform and lifting it to create the half degree of freedom. The original solution was to use a solenoid to lift the platform holding the motors but due to the material used (wood) and the motors/arms the solenoid was not able to lift it reliably. The solenoid could hold the platform but it could not lift it. As a temporary fix a spring was used to reduce some of the downward force of the platform which helped to barely lift the pen but was not fully effective.

	A potential solution to this issue would be to change the material used to make the platform and the lifting mechanism. A scissor lift system using a dc motor would have been a much better choice as it would have allowed for precise control of the height of the platform; however, it would have made moving the platform up and down a little slower but the improved performance would outweigh this downside. As for the material, a 3d printed platform with a light material would allow for better strength and would have reduced the load on the solenoid. A potential problem with this solution however is that the material could have melted due to the heat of the motors.

		

		


### Software

Memory issues:

	One of the main issues with the software for this system was the limitations of the on board memory. Python uses a lot of memory whenever it makes a variable because of the built in overhead for integers. On top of that list in python take a lot of space due to them automatically resizing when new elements are added. This would become an issue whenever a high resolution was used or if more interpolation points were added. To get around this issue points generated were stored within a text file and then read when the arms were to be moved. This would improve the system slightly but memory issues would still occur even when storing values in the flash memory instead of RAM.

	To improve upon this issue I would instead of processing and storing the data on the board I would process the vector image data on the main PC and then store the values found to a file and then store that file within the flash memory of the board. In addition the memory issue could be avoided completely by storing the data on the computer and sending the values serially over UART to the microcontroller. This would slow down the system considerably but would completely circumvent the memory issues.  

Newton-Raphson:

	Calculation of the motor angles using the Newton-Raphson method would sometimes result in an infinite loop because the error of the system would not get below the threshold value.

	

	To fix this issue I would choose a different method to determine the angles of the system. This could result in a slightly less accurate system but would stop the system from getting stuck in an infinite loop. A possible solution would be to use the inverse kinematics of the system to find the angle values given an x and y position. A possible reference is this paper on another pantograph system [http://www.cim.mcgill.ca/~haptic/pub/GC-QW-VH-IROS-05.pdf](http://www.cim.mcgill.ca/~haptic/pub/GC-QW-VH-IROS-05.pdf)


## Conclusions

Overall this system in its current state gives lackluster results and due to the issue encountered did not produce the drawing desired. However, with the improvements mentioned above then the robot would most likely improve greatly over the results that we achieved. And could possibly result in very clean drawings.


