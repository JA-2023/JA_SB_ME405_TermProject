import micropython

from pyb import *


#needed to output error messages
micropython.alloc_emergency_exception_buf(100)

#init/set up
class Driver:
    
    
    def __init__(self, PULSE_DIV, RAMP_DIV):
        #set up pins
        #CS1 = PC2
        self.CS1 = Pin(Pin.cpu.C2, mode=Pin.OUT_PP, value = 1)
        
        #CS2 = PC0
        self.CS2 = Pin(Pin.cpu.C0, mode=Pin.OUT_PP, value = 1)

        #TMC2208 ENN1 = PC3
        TMC_ENN1 = Pin(Pin.cpu.C3, mode=Pin.OUT_PP, value = 1)

        #TMC2208 ENN2 = PB0
        TMC_ENN2 = Pin(Pin.cpu.B0, mode=Pin.OUT_PP, value = 1)

        #CLK = PB3
        CLK = Pin(Pin.cpu.B3, mode=Pin.OUT_PP, value = 1)
        
        #set up SPI
        #SPI(2),(NSS, SCK, MISO, MOSI) = (PB12, PB13, PB14, PB15)
        self.spi = SPI(2, SPI.CONTROLLER, baudrate = 2000000, polarity = 1, phase = 1)
        

        #set up timer/timer channel
        clk_timer = Timer(2, period = 3, prescaler = 0)
        CLK = clk_timer.channel(2, pin = CLK, mode = Timer.PWM, pulse_width = 2)
        
        #enable TMC2208_1
        TMC_ENN1.low()

        #enable TMC2208_2
        TMC_ENN2.low()
        
        #enable TMC4210
        #set en_sd bit in config reg
        TMC4210_data = bytearray([0X68,0x00 ,0x00,0x20])
        
        
        #TMC1 en_sd = 1
        self.send_recv1(TMC4210_data)

        #TMC2 en_sd = 1
        self.send_recv2(TMC4210_data)

        #set veloctiy values for both drivers
        self.set_velocity(0x7FF, 0x00F)
        
        #set clock pre-dividers, PULSE_DIV, RAMP_DIV
        PULE_RMP_BITS = (PULSE_DIV << 4) + RAMP_DIV #this should work
        DIV_data = bytearray([0x18,0x00,PULE_RMP_BITS,0x00])
        
        self.send_recv1(DIV_data)
        self.send_recv2(DIV_data)
        
        #set A_max with pmul and pdiv
        self.set_acceleration(3000, PULSE_DIV, RAMP_DIV) 
        
        
        #set ramp mode
        #set bits 1-0 to 00 at address 0001010 rest are 0s
        RMP_data = bytearray([0x14,0x00,0x00,0x00])

        self.send_recv1(RMP_data)
        self.send_recv2(RMP_data)
        
        
        #set mot1r to 1
        motor_data = bytearray([0x7E,0x20,0x00,0x00])
        
        self.send_recv1(motor_data)
        self.send_recv2(motor_data)
        
        #can set target positions after this is done
        
    #set veloctiy
    def set_velocity(self, V_MAX, V_MIN):
        
        
        #set velocity parameters V_MIN and V_MAX
        V_MIN_upper = (0x700 & V_MIN) >> 8 #gets the last 3 bits of V_MIN
        V_MIN_lower = (0x0FF & V_MIN) #gets the first byte
        VMIN_data = bytearray([0x04, 0x00, V_MIN_upper, V_MIN_lower])
        
        V_MAX_upper = (0x700 & V_MAX) >> 8 #gets last 3 data bits and shifts them by a byte
        V_MAX_lower = (0x0FF & V_MAX) #gets the first byte
        VMAX_data = bytearray([0x06, 0x00, V_MAX_upper, V_MAX_lower])
        
        #send the data to TMC1
        self.send_recv1(VMIN_data)
        self.send_recv1(VMAX_data)

        #send the data to TMC2
        self.send_recv2(VMIN_data)
        self.send_recv2(VMAX_data)

        
    #set acceleration
    def set_acceleration(self, A_MAX, PULSE_DIV, RAMP_DIV):

        #send A_MAX to address 00001100 data in bits 0-10
        A_MAX_upper = (0x0700 & A_MAX) >> 8
        A_MAX_lower = (0x00FF & A_MAX)
        A_MAX_data = bytearray([0x0C, 0x00, A_MAX_upper, A_MAX_lower])

        #send amax to TMC1
        self.send_recv1(A_MAX_data)

        #send amax to TMC2
        self.send_recv2(A_MAX_data)

        #calculate PMUL/PDIV
        calc_vals = self.accel_calc(A_MAX, PULSE_DIV, RAMP_DIV)

        pmul = 0xFF & int(calc_vals[0])
        pdiv = 0x0F & calc_vals[1]

        Pmul_div_data = bytearray([0x12, 0x00, pmul, pdiv])

        #send data to TMC1
        self.send_recv1(Pmul_div_data)

        #send data to TMC2
        self.send_recv2(Pmul_div_data)
        


    #acceleration calc
    def accel_calc(self, A_MAX, PULSE_DIV, RAMP_DIV):

        p = A_MAX / (128.0 * pow(2, (RAMP_DIV - PULSE_DIV)))
        for pmul in range(128,255):
            for p_num in range(3,16):
                
                pdiv = 2**p_num
                p_val = pmul/pdiv
                q = p_val/p
                
                if((0.95 <= q) and (q <= 1.0)):
                    PMUL = pmul
                    PDIV = pdiv
                    break
        
        pmul = int(pmul)
        return pmul, pdiv
        
    #set target position of TMC1
    def set_position1(self, X_TARGET):
        
        #Mask data and shift to set byte array data
        X_byte_1 = (X_TARGET & 0xFF0000) >> 16
        X_byte_2 = (X_TARGET & 0x00FF00) >> 8
        X_byte_3 = (X_TARGET & 0x0000FF)

        X_data = bytearray([0x00,X_byte_1,X_byte_2,X_byte_3])
        self.send_recv1(X_data)
    
    #set target position of TMC2
    def set_position2(self, X_TARGET):
        
        #Mask data and shift to set byte array data
        X_byte_1 = (X_TARGET & 0xFF0000) >> 16
        X_byte_2 = (X_TARGET & 0x00FF00) >> 8
        X_byte_3 = (X_TARGET & 0x0000FF)

        X_data = bytearray([0x00,X_byte_1,X_byte_2,X_byte_3])
        self.send_recv2(X_data)
        
    #get position
    def read_position1(self):
        X_read = bytearray([0x03, 0x00, 0x00, 0x00])
        data_read = bytearray(4)

        #send/recieve data
        data_read = self.send_recv1(X_read) #sends the address and stuff recieved should be the X data

        return data_read

    #get position
    def read_position2(self):
        X_read = bytearray([0x03, 0x00, 0x00, 0x00])
        data_read = bytearray(4)

        #send/recieve data
        data_read = self.send_recv2(X_read) #sends the address and stuff recieved should be the X data

        return data_read

        
    def send_recv1(self, datagram):
        #buffer to hold return values
        rxbuf = bytearray(4)
        #pull CS low to start transmission
        self.CS1.low()
        #send bytes
        self.spi.send_recv(datagram, rxbuf) #hold bytes recieved in variable
        #set chip select high to stop transmission
        self.CS1.high()
        #return data
        return rxbuf

    def send_recv2(self, datagram):
        #buffer to hold return values
        rxbuf = bytearray(4)
        #pull CS low to start transmission
        self.CS2.low()
        #send bytes
        self.spi.send_recv(datagram, rxbuf) #hold bytes recieved in variable
        #set chip select high to stop transmission
        self.CS2.high()
        #return data
        return rxbuf

    





