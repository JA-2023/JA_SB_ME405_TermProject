import micropython

from pyb import *

import gc
import cotask
import taskshare
import hpgl_reading
import project_driver as Driver
import machine
import lcd_api
import i2c_lcd
from machine import Pin, SoftI2C
from lcd_api import LcdApi
from i2c_lcd import I2cLcd
from time import sleep

#disable repl on uart
repl_uart(None)

#needed to output error messages
micropython.alloc_emergency_exception_buf(100)


def task_PenUp():
    while True:
        #check for pen up flag
        if PenUp_flag.get():
            print("in pen up")
            #turn on solenoid
            solenoid.high()
            #make pen up flag low
            PenUp_flag.put(0)
            #make read flag high to read next command
            Read_flag.put(1) 
        yield(0)

def task_PenDown():
    while True:
        #check for pen down flag
        if PenDown_flag.get():
            #turn off solenoid
            solenoid.low()
            #turn off pen down flag
            PenDown_flag.put(0)
            #make read flag high
            Read_flag.put(1)
        yield(0)
    
    
def task_MoveArms():
    while True:
        #check if move_arms flag is high
        if(Move_flag.get()):
            #read theta values from share and then set motor position
            TMC.set_position1(Motor1_angles.get())
            TMC.set_position2(Motor2_angles.get())
            
            #set move flag low
            Move_flag.put(0)

            #make LCD flag high
            LCD_flag.put(1)
        yield(0)


def task_UpdateLCD():
    
    while True:
        if LCD_flag.get():
            #moves cursor to second row (our lcd had issues)
            lcd.move_to(19,2)
            #put blank space at the end to clear 1s place on 3 digit numbers
            lcd.putstr(f" ")
            #move cursor to third row
            lcd.move_to(19,3)
            #put blank space at the end to clear 1s place on 3 digit numbers
            lcd.putstr(f" ")
            #move to beginning of second row and write out motor values
            lcd.move_to(0, 2)
            lcd.putstr(f"motor1 position: {Motor1_text.get()}")
            #move to beginning of third row and write out motor values
            lcd.move_to(0, 3)
            lcd.putstr(f"motor2 position: {Motor2_text.get()}")
            
            #set LCD flag low
            LCD_flag.put(0)

            #set flag to go to read task
            Read_flag.put(1)

        yield(0)

def task_ReadCommands():

    #open file to read motor angle values
    file = open('motor_values.txt','r')
    while True:
        
        if(Read_flag.get()):
            #read line of text from file
            txt1 = file.readline()
            #strip off newline character so it can be converted to number if needed
            txt1 = txt1.strip('\n')

            #will try to check the string read, if there is an index error then the end was reached
            #if the end was reached then stay in read task and close file
            try:
                #check if the text is a motor value
                if txt1[0].isdigit():
                    #convert value into an int and put it into a share
                    Motor1_angles.put(int(float(txt1)))
                    Motor1_text.put(int(float(txt1)))

                    #read next line from file to get second motor value
                    txt2 = file.readline()
                    txt2 = txt2.strip('\n')

                    #convert motor value into an int and put it into a share
                    Motor2_angles.put(int(float(txt2)))
                    Motor2_text.put(int(float(txt2)))

                    #set flag to go to the move task
                    Move_flag.put(1)
                else:
                    #check which command it is and go to the right task
                    if txt1 == 'PU':
                        #set flag high to go to pen up task
                        PenUp_flag.put(1)
                    if txt1 == 'PD':
                        #set flag high to go to pen down task
                        PenDown_flag.put(1)

                #set read flag low so it moves to next task 
                Read_flag.put(0)
            except:
                #idle in read command task
                Read_flag.put(1)
                file.close
        yield(0)

if __name__ == '__main__':
    
    #Initalize driver 
    TMC = Driver.Driver(7,7)

    #parse commands from hpgl file and put it into a text file
    hpgl_reading.parsing('drawing_1.hpgl')

    #make pin to turn solenoid on and off
    solenoid = Pin(Pin.cpu.C4, mode=Pin.OUT_PP, value = 0)
    
    #set up for I2C LCD
    I2C_ADDR = 0x27
    totalRows = 4
    totalColumns = 20
    SDA = Pin(Pin.cpu.B7)
    SCL = Pin(Pin.cpu.B6)
    i2c = SoftI2C(scl=SCL, sda=SDA, freq=5000) 
    lcd = I2cLcd(i2c, I2C_ADDR, totalRows, totalColumns)
    
    
    
    #set up shares
    LCD_flag = taskshare.Share('H', thread_protect = False, name = "LCD_share")
    PenUp_flag = taskshare.Share('H', thread_protect = False, name = "PU_share")
    PenDown_flag = taskshare.Share('H', thread_protect = False, name = "PD_share")
    Move_flag = taskshare.Share('H', thread_protect = False, name = "Move_share")
    Read_flag = taskshare.Share('H', thread_protect = False, name = "Read_share")
    command_counter = taskshare.Share('H', thread_protect = False, name = "Count_share")
    Motor1_angles = taskshare.Share('H', thread_protect = False, name = "angle1_share")
    Motor2_angles = taskshare.Share('H', thread_protect = False, name = "angle2_share")
    Motor1_text = taskshare.Share('H', thread_protect = False, name = "text1_share")
    Motor2_text = taskshare.Share('H', thread_protect = False, name = "text2_share")
 
    #set up tasks
    PD_task = cotask.Task(task_PenDown, name = 'button_task', priority = 2,
                              period = 150, profile = True, trace = False)
    
    PU_task = cotask.Task(task_PenUp, name = 'button_task', priority = 2,
                              period = 150, profile = True, trace = False)

    Read_task = cotask.Task(task_ReadCommands, name = 'button_task', priority = 3,
                              period = 150, profile = True, trace = False)

    LCD_task = cotask.Task(task_UpdateLCD, name = 'button_task', priority = 1,
                              period = 100, profile = True, trace = False)

    Move_task = cotask.Task(task_MoveArms, name = 'button_task', priority = 3,
                              period = 100, profile = True, trace = False)                          
                          
    
    #add task to list
    cotask.task_list.append(PD_task)
    cotask.task_list.append(PU_task)
    cotask.task_list.append(Read_task)
    cotask.task_list.append(LCD_task)
    cotask.task_list.append(Move_task)
    
    gc.collect()
    
    #start in read task
    Read_flag.put(1)

    while True:
        cotask.task_list.pri_sched()

    




    

