#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Byte
from std_msgs.msg import Bool
from std_msgs.msg import ByteMultiArray

from Phidget22.Phidget import *
from Phidget22.Devices.DigitalOutput import *
from Phidget22.Devices.DigitalInput import *
import time

from phidget_digital_interface.msg import inputs_outputs
from phidget_digital_interface.srv import set_digital_output, set_digital_outputResponse


stateDigitalPins  = inputs_outputs()
stateDigitalPins.digital_inputs = []
stateDigitalPins.digital_outputs = []

for i in xrange(0,4): stateDigitalPins.digital_inputs.append(0)
for i in xrange(0,4): stateDigitalPins.digital_outputs.append(0)


def digital_output_callback(request): #Set the digital outputs

    channel = request.output
    state = request.value
    stateDigitalPins.digital_outputs.pop(channel)
    stateDigitalPins.digital_outputs.insert(channel,state)
    print("State output channel " + str(channel) + ": " + str(stateDigitalPins.digital_outputs[channel]))
    my_response = set_digital_outputResponse()
    my_response.ret = True
    return my_response


def onStateChange(self, state):

	global stateDigitalPins

	channel = self.getChannel()

	stateDigitalPins.digital_inputs.pop(channel)
	stateDigitalPins.digital_inputs.insert(channel,state)

	if channel < 3:	
		print("State input channel " + str(channel) + ": " + str(stateDigitalPins.digital_inputs[channel]))





#This code uses Digital Output 1100_0 Phidget model and set channel 0, 1, 2 and 3 as digital output. 
#This code uses Digital Input 1300_0 Phidget model and set channel 0, 1, 2 and 3 as digital inputs.
#Model 1100_0 is connected with the channel 0 from the VINT hub and model 1300_0 is connected with
#channel 1 from the VINT hub.


# -------------------------------------------- ROS Configuration ---------------------------------------------#


rospy.init_node('phidget_digital_interface')


pub_inputs_outputs_state = rospy.Publisher('/phidget/inputs_outputs/getState', inputs_outputs , queue_size=1)

service_digital_output = rospy.Service('/phidget/digital_output/setState', set_digital_output , digital_output_callback)

rate = rospy.Rate(10)

# -------------------------------------------- Pidghet Configuration ---------------------------------------------#



#Create your Phidget channels
digitalOutput0 = DigitalOutput()
digitalOutput1 = DigitalOutput()
digitalOutput2 = DigitalOutput()
digitalOutput3 = DigitalOutput()

digitalInput0 = DigitalInput()
digitalInput1 = DigitalInput()
digitalInput2 = DigitalInput()
digitalInput3 = DigitalInput()


#Set addressing parameters to specify which channel to open (if any)
digitalOutput0.setHubPort(0)
digitalOutput0.setChannel(0)
digitalOutput1.setHubPort(0)
digitalOutput1.setChannel(1)
digitalOutput2.setHubPort(0)
digitalOutput2.setChannel(2)
digitalOutput3.setHubPort(0)
digitalOutput3.setChannel(3)

digitalInput0.setHubPort(1)
digitalInput0.setChannel(0)
digitalInput1.setHubPort(1)
digitalInput1.setChannel(1)
digitalInput2.setHubPort(1)
digitalInput2.setChannel(2)
digitalInput3.setHubPort(1)
digitalInput3.setChannel(3)


#Assign any event handlers you need before calling open so that no events are missed.
digitalInput0.setOnStateChangeHandler(onStateChange)
digitalInput1.setOnStateChangeHandler(onStateChange)
digitalInput2.setOnStateChangeHandler(onStateChange)
digitalInput3.setOnStateChangeHandler(onStateChange)


#Open your Phidgets and wait for attachment
digitalOutput0.openWaitForAttachment(5000)
digitalOutput1.openWaitForAttachment(5000)
digitalOutput2.openWaitForAttachment(5000)
digitalOutput3.openWaitForAttachment(5000)

digitalInput0.openWaitForAttachment(5000)
digitalInput1.openWaitForAttachment(5000)
digitalInput2.openWaitForAttachment(5000)
digitalInput3.openWaitForAttachment(5000)


#By default digital outputs are 0
digitalOutput0.setDutyCycle(0)
digitalOutput1.setDutyCycle(0)
digitalOutput2.setDutyCycle(0)
digitalOutput3.setDutyCycle(0)


stateDigitalOutput0 = 0
stateDigitalOutput1 = 0
stateDigitalOutput2 = 0
stateDigitalOutput3 = 0


# -------------------------------------------- Loop code  ---------------------------------------------#

while not rospy.is_shutdown():
	
  #Publishs digital inputs and outputs states
  pub_inputs_outputs_state.publish(stateDigitalPins)
 
  #Refresh output states in the Phidgets
  digitalOutput0.setDutyCycle(stateDigitalPins.digital_outputs[0])
  digitalOutput1.setDutyCycle(stateDigitalPins.digital_outputs[1])
  digitalOutput2.setDutyCycle(stateDigitalPins.digital_outputs[2])
  digitalOutput3.setDutyCycle(stateDigitalPins.digital_outputs[3])

  rate.sleep()


#Close your Phidgets once the program is done.
digitalOutput0.close()
digitalOutput1.close()
digitalOutput2.close()
digitalOutput3.close()

digitalInput0.close()
digitalInput1.close()
digitalInput2.close()
digitalInput3.close()




