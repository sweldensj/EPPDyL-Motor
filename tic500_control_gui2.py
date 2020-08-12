"""
Last update: 3 August 2020
Adapted from tic500_motor_control_gui.py in 2020

User-friendly control interface for the Tic T500 Stepper Motor Controller
for the EPPDyL.

@author: Johan Sweldens
Created in Summer 2020
email: jws426@cornell.edu 

Original Code was made by Sydney Hsu from Spring 2020
"""

# -----------------------------------------------------------------------------
    # Import useful packages
# -----------------------------------------------------------------------------
from tkinter import Tk,Toplevel,Menu,PhotoImage,StringVar,N,S,E,W,FALSE,filedialog
from tkinter import ttk
import serial
import numpy as np
import time

#----------------------------------------------------------------
    # Creating the Tic object and setting its functions
#----------------------------------------------------------------
class TicSerial(object):
    def __init__(self, port, device_number=None, myHome = 0):
        self.port = port
        self.device_number = device_number
        self.myHome = myHome
        print("Contact made with these parameters...")
        print("Port: " + str(self.port))
        print("Device Number: " + str(self.device_number))
        print("Home Value: " + str(self.myHome))

    #Updates the Home value to be displayed on the interface
    def update_Home(self, newHome):
        self.myHome = newHome
        #print("DEBUG: Update Home Value" + str(self.myHome))
    
    #For more information on the send_command, please refer to https://www.pololu.com/docs/0J71/8#cmd-get-variable
    def send_command(self, cmd, *data_bytes): 
        if self.device_number == None:
            header = [cmd]  # Compact protocol
        else:
            header = [0xAA, device_number, cmd & 0x7F]  # Pololu protocol
        self.port.write(bytes(header + list(data_bytes)))
 
    # Sends the "Exit safe start" command.
    def exit_safe_start(self):
        self.send_command(0x83)

    # Sets the target position.
    # For more information about what this command does, see the
    # "Set target position" command in the "Command reference" section of the
    # Tic user's guide.
    def set_target_position(self, target):
        self.send_command(0xE0,
            ((target >>  7) & 1) | ((target >> 14) & 2) |
            ((target >> 21) & 4) | ((target >> 28) & 8),
            target >> 0 & 0x7F,
            target >> 8 & 0x7F,
            target >> 16 & 0x7F,
            target >> 24 & 0x7F)
 
    # Gets one or more variables from the Tic. https://www.pololu.com/docs/0J71/7
    def get_variables(self, offset, length):
        self.send_command(0xA1, offset, length)
        result = self.port.read(length)
        if len(result) != length:
            raise RuntimeError("Expected to read {} bytes, got {}."
                .format(length, len(result)))
        return bytearray(result)
 
    # Gets the "Current position" variable from the Tic.
    def get_current_position(self):
        b = self.get_variables(0x22, 4)
        position = b[0] + (b[1] << 8) + (b[2] << 16) + (b[3] << 24)
        if position >= (1 << 31):
            position -= (1 << 32)
        return position

    #Get limit switch value. 
    def get_limit_switch(self):
        checkLimited = self.get_variables(0x47, 2) #Get RC Pin value. #Change length if need be. 2 should normally work
        #print(type(checkLimited)) #Very useful in debuging the RC Pin
        #print(checkLimited) #Very useful in debuging RC PIN
        if (checkLimited == bytearray(b'\x17z')  or checkLimited == bytearray(b'\x1fz')): #May vary with computer. 
            print('Limit Reached')
            return True
        else :
            print('Have not reached limit')
            return False

#----------------------------------------------------------------
    #SETTINGS
#----------------------------------------------------------------
#Choose the serial port name. !!!!THIS NEEDS TO BE CHANGED DEPENDING ON THE COMPUTER!!!!!!
#Windows use "COMx" where x is a number between 2 and 5. Most common is 3 and 4 ex "COM3"
#Linux and Macs use things like "/dev/ttyACM0" or "/dev/ttyUSB0"
#Once everything is settup use this function in command prompt to see which port you need to use
#   python -m serial.tools.list_ports
port_name = "COM4" 
 
# Choose the baud rate (bits per second).  This must match the baud rate in
# the Tic's serial settings. 9600 is default
baud_rate = 9600
 
# Change this to a number between 0 and 127 that matches the device number of
# your Tic if there are multiple serial devices on the line and you want to
# use the Pololu Protocol. None is default
device_number = None

#Home value. This will get changed once the calibration function is called. 
defHome = 0 #default value.  

#motor maximum speed Change first value only! 
motorMaxSpeed = 5000 #STEPS PER SECOND, Default value is 200 steps per second. Not recommened to exceed this
motorMaxSpeed = motorMaxSpeed * 10000 #Converting to the system the motor uses. Ignore this line

#maximum amount of seconds the calibration function is allowed to run before its shutdown. 
#preventative measure incase of physical problem with limit switch. 
calibrationTime = 30

#Amount of steps away from the limit switch that the motor will move too. 
calibrationOffset = 20

#Establishing the serial port.
port = serial.Serial(port_name, baud_rate, timeout=0.1, write_timeout=0.1)
 
#Establishing the tic as an object 
tic = TicSerial(port, device_number, defHome)
 



# -----------------------------------------------------------------------------
    # Class to more easily manage ttk.Label objects with associated
    # StringVars.
    # Can set associated variable by setting the TrackedLabel.value property,
    # or using call syntax.
# -----------------------------------------------------------------------------
class TrackedLabel(ttk.Label):

    def __init__(self,*args,**kwargs):
        self._tracked_var = StringVar()
        self._tracked_var.set(kwargs['text'])
        kwargs['textvariable']=self._tracked_var
        ttk.Label.__init__(self,*args,**kwargs)

    @property
    def value(self):
        return self._tracked_var.get()

    @value.setter
    def value(self,val):
        self._tracked_var.set(val)

    def __call__(self,val):
        self.value = val

class MotorControl(Tk):
    '''
    Tk root object, with controls for motor.
    Opens probe controls. This Setups the interface. 
    '''
    def __init__(self):
        Tk.__init__(self)
        self.option_add('*tearOff',FALSE)
        self.title('Plasma Diagnostics Probe Control')

        #create variable to hold motor object once instantiated
        self.m = None

        # GUI frame
        self.gui_frame = ttk.Frame(self,padding=10)
        self.gui_frame.grid(row=0,column=0)

        # Row 0: Motor address and connection status
        self.motor_port_label = ttk.Label(self.gui_frame,text='Port address:')
        self.motor_port = StringVar()
        self.motor_port.set(port_name)
        self.motor_port_entry = ttk.Entry(self.gui_frame,textvariable=self.motor_port,width=7)
        self.connect_button = ttk.Button(self.gui_frame,text='Connect',command=self.attempt_connection,width=10)
        self.connect_status = TrackedLabel(self.gui_frame,text='No connection',width=17)

        # Manual controls section
        self.motor_control_label = ttk.Label(self.gui_frame,text='Manual Motor Controls')

        # number of steps entry field
        self.step_label = ttk.Label(self.gui_frame,text='Steps: ')
        self.stepnumber = StringVar()
        self.stepnumber.set('0')
        self.step_entry = ttk.Entry(self.gui_frame,textvariable=self.stepnumber,width=10)

        # speed entry field
        self.speed_label = ttk.Label(self.gui_frame,text='Speed: ')
        self.speednumber = StringVar()
        self.speednumber.set('200')
        self.speed_entry = ttk.Entry(self.gui_frame,textvariable=self.speednumber,width=10)
        self.set_speed_button = ttk.Button(self.gui_frame,text='Set speed',command=self.motor_set_speed,width=10)

        # buttons
        self.fwd_button = ttk.Button(self.gui_frame,text='Forward',command=self.motor_fwd_step,width=10)
        self.rev_button = ttk.Button(self.gui_frame,text='Reverse',command=self.motor_rev_step,width=10)
        self.stop_button = ttk.Button(self.gui_frame,text='Stop',command=self.motor_stop,width=10)
        self.homing_button = ttk.Button(self.gui_frame,text='Go home',command=self.motor_go_home,width=10)
        self.calibrate_button = ttk.Button(self.gui_frame,text='Calibrate',command=self.motor_calibrate,width=10)

        # Live position readout
        self.position_label = ttk.Label(self.gui_frame,text='Current position:')
        self.current_position = TrackedLabel(self.gui_frame,text='(?)')

        # Assemble GUI
        # Motor port
        self.motor_port_label.grid(row=0,column=0,sticky=E)
        self.motor_port_entry.grid(row=0,column=1,columnspan=4,sticky=W+E)
        self.connect_button.grid(row=0,column=5,sticky=W+E)
        self.connect_status.grid(row=0,column=6,columnspan=2,sticky=W)

        # Manual control section
        self.motor_control_label.grid(row=1,column=2,columnspan=3)
        self.step_label.grid(row=3,column=0,sticky=E)
        self.step_entry.grid(row=3,column=1,sticky=W+E)

        self.speed_label.grid(row=4,column=0,sticky=E)
        self.speed_entry.grid(row=4,column=1,sticky=W+E)
        self.set_speed_button.grid(row=4,column=2,sticky=W+E)

        self.fwd_button.grid(row=3,column=2,sticky=W+E)
        self.rev_button.grid(row=3,column=3,sticky=W+E)
        self.stop_button.grid(row=3,column=4,sticky=W+E)
        self.homing_button.grid(row=4,column=3,sticky=W+E)
        self.calibrate_button.grid(row=4,column=4,sticky=W+E)

        # Current position readout
        self.position_label.grid(row=5,column=6,sticky=E)
        self.current_position.grid(row=5,column=7)

        #bindings, hotkeys, etc.
        self.bind('<Control-w>',self.close)
        self.after(100,self.update_position)

# -----------------------------------------------------------------------------
    # Direct serial commands to motor via tic object 
# -----------------------------------------------------------------------------

    #Stopping and Deenergizing the motor. 
    def motor_stop(self):
        print('Breaking...')
        tic.send_command(0x92) #Stop motor
        time.sleep(1)
        tic.send_command(0x86) #Deenergize
        print('Deenergized Motor')



    #Calibration. This calibration method is very different from the original. 
    #This just depends upon the limit switch for a positional calibration. 
    def motor_calibrate(self):
        print('Calibrating...')
        tic.exit_safe_start()
        #Move onestep backward at a time,
        #Constantly check if the switch is pressed
        limitReached = tic.get_limit_switch() #Boolean 
        startTime = time.time() #Keep track of how long the calibration process has been going on for
        continueLoop = (time.time() - startTime <= calibrationTime) and not limitReached
        while continueLoop:
            position = tic.get_current_position()
            new_target = position - 3 #Feel free to change if its not smooth enough
            tic.set_target_position(new_target)
            time.sleep(0.15) #change if not smooth enough
            limitReached = tic.get_limit_switch()
            continueLoop = (time.time() - startTime <= calibrationTime) and not limitReached
            #print("DEBUG: Calibration Iteration")
            #print(time.time() - startTime)
        #print("DEBUG: Calibration loop completed")

        #If switch is pressed, move forward by Offset
        if limitReached:
            new_target = tic.get_current_position() + calibrationOffset #change offset as need be
            tic.set_target_position(new_target)
            #set new position as home. 
            print(new_target)
            tic.update_Home(new_target) #Change displayed position value to match calibration
            print('Calibrated!')
        else:
            print('Calibration Failed, did not interface with limit switch')
            self.motor_go_home() #Go back home in failure
        

    #connect to port
    def attempt_connection(self):
        try:
            #print("DEBUG: Attempting Connection.")
            self.connect_status('Connecting....')
            tic.exit_safe_start()
            print("Energized motor")
            tic.send_command(0x85) #Energize motor
            self.connect_status('Connected')
        except:
            print("DEBUG: Unable to connect")
            self.connect_status('Failed')
        

    # go forward by k steps
    def motor_fwd_step(self):
        '''
        Step size can be negative. It will go in the opposite direction 
        '''
        #print('DEBUG: Moving Foward')
        position = tic.get_current_position()
        new_target = position + int(self.stepnumber.get())
        print("Setting target position to {}.".format(new_target));
        tic.exit_safe_start()
        tic.set_target_position(new_target)
        

    # go in reverse by k steps
    def motor_rev_step(self):
        '''
        Step size can be negative. It will go in the opposite direction  
        '''
        #print('DEBUG: Moving Backward')
        position = tic.get_current_position()
        new_target = position - int(self.stepnumber.get())
        print("Setting target position to {}.".format(new_target));
        tic.exit_safe_start()
        tic.set_target_position(new_target)


    #Go back to the home value. This needs to bypass the timed shutdown. Thats why theres a recursive loop
    def motor_go_home(self):
        #print("DEBUG: Going home")
        target = tic.myHome #get the home value from the tic object class
        tic.exit_safe_start()
        tic.set_target_position(target)
        position = tic.get_current_position()
        if not (position == target):
            #print('DEBUG: Recursion in go home')
            self.motor_go_home()
        #print("DEBUG: At Home")


    # set speed
    def motor_set_speed(self):
        
        mySpeed = self.speednumber.get() #Gets speed inputed from interface
        print("Speed set to {} Hz".format(str(mySpeed)))
        speed = int(mySpeed) #Change into integer
        speed = speed * 10000 #200,0000
        if (speed > motorMaxSpeed | speed < 0): # don't let speed exceed bounds
            speed = motorMaxSpeed 
        # encode speed into bytes. Look into byte encoding if this is confusing
        tic.send_command(0xE6,((speed >>  7) & 1) | ((speed >> 14) & 2) |
            ((speed >> 21) & 4) | ((speed >> 28) & 8),
            speed >> 0 & 0x7F,
            speed >> 8 & 0x7F,
            speed >> 16 & 0x7F,
            speed >> 24 & 0x7F)


    # position ranges from 0 to 100, calibrated against voltage
    def update_position(self,repeat=True):
        #print(tic.myHome)
        position = tic.get_current_position() - tic.myHome #Change by offset once the motor has been calibrated
        #print("Current position is {}.".format(position))
        self.current_position('{:4.2f} Steps'.format(position*1)) #Change to fit position scheme. 
        #Changed 'cm' to 'Steps and '*100' to '*1'. Edit as need be 
        if repeat: #If repeating, wait once second before updating again. 
            self.after(200,self.update_position)

    def close(self,*args):
        if self.m is not None:
            del(self.m)
        self.destroy()

if __name__=='__main__':
    root = MotorControl()
    root.mainloop()
