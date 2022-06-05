from ulab import numpy as np
import math
from cmath import pi
import array
import gc


#function to multiple a 4x1 matrix with a 4x4 matrix
def matmul(matrix1, matrix2):
    result = array.array('f',[0,0,0,0])
    for i in range(4): #go through rows matrix 1
        for k in range(4): #go through column of matrix 2
            result[i] += matrix1[i][k] * matrix2[k]
                
    return result

#function to return the jacobian of the system
def dg_dtheta(theta):
    #arm length is 150mm
    l1 = l2 = l3 = l4 = 150
    dg_dtheta = np.array([[l1*math.sin(theta[0]),l2*math.sin(theta[1]), 0, 0],
                        [-l1*math.cos(theta[0]), -l2*math.cos(theta[1]), 0, 0],
                        [0 , 0, l3*math.sin(theta[2]), l4*math.sin(theta[3])],
                        [0, 0, -l3*math.cos(theta[2]), -l4*math.cos(theta[3])]])
    return dg_dtheta

#calculates the angle values of the system in radians
def NewtwonRaphson(fcn, jacobian, guess, thresh):
    theta = guess
    error = 1
    while (error > thresh):
        theta_n = array.array('f',[a - b for a,b in zip(theta,matmul(np.linalg.inv(jacobian(theta)),fcn(theta)))])
        error = np.linalg.norm(fcn(theta_n))
        theta = theta_n
    return theta_n

#calculates the difference between the destination and the system given a theta array
def g(x, theta):
    #g = x-f(theta)
    #x and theta are vectors
    f_theta = np.array([150*math.cos(theta[0]) + 150*math.cos(theta[1]),
                        150*math.sin(theta[0]) + 150*math.sin(theta[1]),
                        150*math.cos(theta[2]) + 150*math.cos(theta[3]) + 135,
                        150*math.sin(theta[2]) + 150*math.sin(theta[3])])
    g = [a - b for a,b in zip(x,f_theta)]
    return g

#takes a list of x coordinates and y coordinates and interpolates them.
def inter_scale_off(x_cords, y_cords):

    inter_x_vals = array.array('f')
    inter_y_vals = array.array('f')
    #variable to increase the number of interpolation points
    scale_points = 10

    for place,cord in enumerate(x_cords):
        if(place < (len(x_cords) - 1)):
            #find distance between two points
            diff = x_cords[place + 1] - cord
            y_diff = y_cords[place + 1] - y_cords[place]
            #find distance between points
            dist = np.sqrt(diff**2 + y_diff**2)/25.4 #puts it in inches

            #multiply the distance by the dpi
            seg_num = int(scale_points*dist) 

            #put current x and y values in list
            inter_x_vals.append(cord)
            inter_y_vals.append(y_cords[place])
            x_calc_val = cord

            #divide by zero check
            if(seg_num != 0):
                x_dist = diff/seg_num
                y_dist = y_diff/seg_num

            #checks if the slope is zero
            if diff != 0:
                for num in range(seg_num):
                    #calculate x value
                    x_calc_val += x_dist
                    #calculate y value with formula
                    y_calc_val = y_cords[place] + ((y_cords[place +1] - y_cords[place]) / (x_cords[place + 1] - cord))*(x_calc_val - cord)
                    #insert new point into list
                    inter_x_vals.append(round(x_calc_val, 1))
                    inter_y_vals.append(round(abs(y_calc_val), 1))

            #checks if it is a vertical line and adds points
            y_val = y_cords[place]
            if (diff == 0) and (abs(y_diff) > 10):
                for num in range(seg_num):
                    #increment be the distance found
                    y_val += y_dist
                    #append original X value so that it matches
                    inter_x_vals.append(cord)
                    inter_y_vals.append(y_val)
                


    #add offset of system and scale if needed
    for ((count, x_cord), y_cord) in zip(enumerate(inter_x_vals), inter_y_vals):
        #scale values
        #x_cord *= 2
        #y_cord *= 2

        #offset values
        x_cord += 67.5 #value is in mm
        y_cord += 196.8 #value in mm to center of page

        #update values with the offset values
        inter_x_vals[count] = round(x_cord,5)
        inter_y_vals[count] = round(y_cord, 5)

    #return the two lists
    return inter_x_vals, inter_y_vals
    
def parsing(file):
    #open file and read values
    with open(file) as drawing:
        info = drawing.readline() #get commands from drawing


    #seperate commands
    seperated = info.split(';') #separate commands 
    fixed_commands = []
    
    #delete array once it is used to save memory
    del(info)
    gc.collect()
    
    #seperate command from first number
    for command in seperated:
        if len(command) > 2: #check if there is a number after the command
            command = command[:2] + ',' + command[2:]
        if(command != ''): #checks if there is a white space
            fixed_commands.append(command)
    
    #delete array once it is used to save memory
    del(seperated)
    gc.collect()
    
    
    #seperate cooridinates from commands
    seperated_commands = []
    for command in fixed_commands:
        seperated_commands.append(command.split(','))
    
    #delete array once it is used to save memory
    del(fixed_commands)
    gc.collect()

    #filter out commands except for PU and PD
    filtered_commands = [command for command in seperated_commands if (command[0] == 'PU' or command[0] == 'PD')]
    
    #delete array once it is used to save memory
    del(seperated_commands)
    gc.collect()
    
    #make list to hold command, x-values, y-values
    parsed_list = []
    #goes through the filtered command and puts it into the format: command, x values, y values
    for command in filtered_commands:
        #check if length is longer >= 1 (filters out initial PU)
        if len(command) > 1:
            #make new list main list
            command_x_y_list = []
            #make new list for x values
            command_x = array.array('I')
            #make new list for y values
            command_y = array.array('I')
            #add command to main list
            command_x_y_list.append(command[0])
            
            #go through each coordinate seperating out x and y values
            for count, val in enumerate(command):

                #y cooridinates are even values (starting at 2)
                if ((count != 0) and (count % 2 == 0)):
                    command_y.append(int(val))

                #x cooridinates are odd values
                elif(count != 0):
                    command_x.append(int(val))

            #add list of x values to command list
            command_x_y_list.append(command_x)
            #add list of y values to command list
            command_x_y_list.append(command_y)
            #put entry in the final list
            parsed_list.append(command_x_y_list)

    #delete array once it is used to save memory        
    del(filtered_commands)
    del(command_x_y_list)
    del(command_x)
    del(command_y)
    gc.collect()
    
    #interpolate, scale, and add the offset to the commands
    for values in parsed_list:
        if ((len(values[1]) > 1) and len(values[2]) > 1):
            values[1], values[2] = inter_scale_off(values[1], values[2])

    final_commands = [parsed_list[2], parsed_list[3], parsed_list[4]]

    # #change first pen up command to be where the start of the drawing is
    final_commands[0][1] = final_commands[1][1][0]
    final_commands[0][2] = final_commands[1][2][0]

    # #change the last pen up to be where it started
    final_commands[2][1] = final_commands[1][1][0]
    final_commands[2][2] = final_commands[1][2][0]

    del(parsed_list)
    gc.collect()
    #-------------------------------------------------------------------


    #----------------------------------------Newton-Raphson--------------------------------
    #make guess on angles
    theta = array.array('f',[90*(np.pi/180), 30*(np.pi/180), 100*(np.pi/180), 60*(np.pi/180)])
    #clear file contents before writing
    open('motor_values.txt','w').close()
    #open file in appending mode
    print('writing to file')
    with open('motor_values.txt','a') as file:

        #put value throug newton raphson
        x_des = array.array('f',[final_commands[0][1],final_commands[0][2],final_commands[0][1],final_commands[0][2]])
        #find theta values for destnation value
        theta_n = NewtwonRaphson(lambda theta: g(x_des,theta),dg_dtheta,theta,1e-4)
        #write command
        file.write(f"{final_commands[0][0]}\n")
        #write motor values
        file.write(f"{(theta_n[0] * (180/pi)) * (256/360)}\n")
        file.write(f"{(theta_n[3] * (180/pi)) * (256/360)}\n")

        #write pen down command to file
        file.write(f"{final_commands[1][0]}\n")

        #goes through each positional value in big pen down command
        for num in range(len(final_commands[1][1])):
            #input x and y values of drawing into destination array
            x_des = array.array('f',[final_commands[1][1][num],final_commands[1][2][num],final_commands[1][2][num],final_commands[1][2][num]])
            #find theta values for destnation value
            theta_n = NewtwonRaphson(lambda theta: g(x_des,theta),dg_dtheta,theta,1e-4)
            #put step values in list
            file.write(f"{(theta_n[0] * (180/pi)) * (256/360)}\n")
            file.write(f"{(theta_n[3] * (180/pi)) * (256/360)}\n")


        #write pen up command to file
        file.write(f"{final_commands[2][0]}\n")
        #input x and y values of circle into destination array
        x_des = array.array('f',[final_commands[2][1],final_commands[2][2],final_commands[2][1],final_commands[2][2]])
        #find theta values for destnation value
        theta_n = NewtwonRaphson(lambda theta: g(x_des,theta),dg_dtheta,theta,1e-4)
        #put theta values in text file
        file.write(f"{(theta_n[0] * (180/pi)) * (256/360)}\n")
        file.write(f"{(theta_n[3] * (180/pi)) * (256/360)}\n")


