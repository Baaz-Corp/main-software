from visual import *
import thread, time, sys, traceback

#---------- SETTINGS -------------- 
use_real_robot = True # Set to True to use data from the COM port, False to use demo data.

com_port = "COM4"  # example: 5 == "COM6" == "/dev/tty5"
baudrate = 115200
#---------------------------------

# serial port
ser = None  # To be initialised

def read_v_2_4():  # Reads serial data and prints
    global init_level, angle, index
    print("in read")
    nb_errors = 0
    while True:  # Infinite loop
        try:
            time.sleep(0.00001) # do not hog the processor power

            if init_level == 0 : # 0
                b = ord(ser.read(1))  # Read first bit
                if b == 0xFA :        # If b is start bit
                    init_level = 1    # set init 1
                else:
                    init_level = 0    # Else stays same
                    
            elif init_level == 1: # 1
                b = ord(ser.read(1))
                if b >= 0xA0 and b <= 0xF9  : # 160 < b < 249
                    index = b - 0xA0
                    init_level = 2
                elif b != 0xFA:
                    init_level = 0
                    
            elif init_level == 2 : # 2
                # "ord()" returns integer that represents character
                
                # get rotation speed
                b_speed = [ ord(b) for b in ser.read(2)]
                
                # get data
                b_data0 = [ ord(b) for b in ser.read(4)]
                b_data1 = [ ord(b) for b in ser.read(4)]
                b_data2 = [ ord(b) for b in ser.read(4)]
                b_data3 = [ ord(b) for b in ser.read(4)]

                # for the checksum, we need all the data of the packet...
                # this could be collected in a more elegent fashion... 
                all_data = [ 0xFA, index+0xA0 ] + b_speed + b_data0 + b_data1 + b_data2 + b_data3 

                # checksum  
                b_checksum = [ ord(b) for b in ser.read(2) ]
                incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)

                # verify that the received checksum is equal to the one computed from the data
                if checksum(all_data) == incoming_checksum:
                    speed_rpm = compute_speed(b_speed)



                    # Form strings with data
                    dat0 = "Angle: %s, distance: %s"%(index * 4 + 0, b_data0[0])
                    dat1 = "Angle: %s, distance: %s"%(index * 4 + 1, b_data1[0])
                    dat2 = "Angle: %s, distance: %s"%(index * 4 + 2, b_data2[0])
                    dat3 = "Angle: %s, distance: %s"%(index * 4 + 3, b_data3[0])

                    # print angle and distance
                    print(dat0)
                    print(dat1)
                    print(dat2)
                    print(dat3)
               # else:
                    # the checksum does not match, something went wrong...
               #     nb_errors +=1
                #    label_errors.text = "errors: "+str(nb_errors)
                    
                    # display the samples in an error state
                    #update_view(index * 4 + 0, [0, 0x80, 0, 0])
                    #update_view(index * 4 + 1, [0, 0x80, 0, 0])
                    #update_view(index * 4 + 2, [0, 0x80, 0, 0])
                    #update_view(index * 4 + 3, [0, 0x80, 0, 0])
                    
                init_level = 0 # reset and wait for the next packet
                
            else: # default, should never happen...
                init_level = 0
        except :
            traceback.print_exc(file=sys.stdout)


 
def checksum(data):
    """Compute and return the checksum as an int.

    data -- list of 20 bytes (as ints), in the order they arrived in.
    """
    # group the data by word, little-endian
    data_list = []
    for t in range(10):
        data_list.append( data[2*t] + (data[2*t+1]<<8) )
    
    # compute the checksum on 32 bits
    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d

    # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
    checksum = checksum & 0x7FFF # truncate to 15 bits
    return int( checksum )





init_level = 0
index = 0

def motor_control( speed ):
    global ser, controler, rpm_setpoint
    val = controler.process( speed - rpm_setpoint)
    ser.write(chr(val))


def compute_speed(data):
    speed_rpm = float( data[0] | (data[1] << 8) ) / 64.0
    return speed_rpm

class pid_controler:
    def __init__(self, kp, ki, kd, ):
        self.set_coeffs( kp, ki, kd)
        self.reset()

    def set_coeffs(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

    def set_max_input(self, max_in):
        self.max_input = max_in
        
    def set_max_output(self, max_out):
        self.max_output = max_out
        
    def set_max_integral(self, max_i):
        self.max_I = max_i
        
    def set_output_ratio(self, ratio):
        self.out_ratio = r

    def reset(self):
        self.integral = 0.0
        self.prev_D = 0.0
        self.prev_sample = 0.0

    def process(error):
        ##  /* derivate value                                             
        ##  *             f(t+h) - f(t)        with f(t+h) = current value
        ##  *  derivate = -------------             f(t)   = previous value
        ##  *                    h
        ##  * so derivate = current error - previous error
        ##  *
        ##  * We can apply a filter to reduce noise on the derivate term,
        ##  * by using a bigger period.
        ##  */
        derivate = error - self.prev_sample

        if self.max_input :
            S_MAX(error, self.max_input) #saturate input before calculating integral
            
        ##   /* 
        ##   * Integral value : the integral become bigger with time .. (think
        ##   * to area of graph, we add one area to the previous) so, 
        ##   * integral = previous integral + current value
        ##   */
        self.integral += error ;

        if self.max_I:
            S_MAX(self.integral, self.max_I); # saturate integrale term

        output =  (long)(self.Kp * ( error + self.Ki * self.integral + (self.Kd * derivate) / PID_DERIVATE_FILTER_SIZE ));

        output = self.out_ratio * output;
  
        if(self.max_output):
            S_MAX(output, self.max_output);  # saturate output
    
        # backup values for the next calcul of derivate */
        self.prev_sample = error;
        self.prev_D = derivate
  
        return output
           
# Main program
if use_real_robot:
    import serial
    ser = serial.Serial(com_port, baudrate)
    print("in main")
    th = thread.start_new_thread(read_v_2_4, ())

while True:
    rate(5) # synchonous repaint at 60fps
    
