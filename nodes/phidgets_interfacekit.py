#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('phidgets')
import rospy

from std_msgs.msg import String

from phidgets.srv import SrvPhidgetsInterfaceKitGetAI, SrvPhidgetsInterfaceKitGetAIResponse
from phidgets.srv import SrvPhidgetsInterfaceKitGetDI, SrvPhidgetsInterfaceKitGetDIResponse
from phidgets.srv import SrvPhidgetsInterfaceKitSetDO, SrvPhidgetsInterfaceKitSetDOResponse

import Phidgets
import Phidgets.Devices.InterfaceKit
from setdict import SetDict


###############################################################################
###############################################################################
# PhidgetsInterfaceKit()
# Provides ROS access to the digital and analog i/o of the PhidgetsInterfaceKit.
# Three service calls:
#   get_ai(channels):         Read the analog input channels, publish the voltages
#                             to the ROS topic 'ai', and return the results.
#   get_di(channels):         Read the digital input channels, publish the voltages
#                             to the ROS topic 'di', and return the results.
#   set_do(channels,values):  Set the digital output channels to the given values.
#
class PhidgetsInterfaceKit:

    def __init__(self):
        self.bInitialized = False
        self.iCount = 0
        self.bAttached = False
        self.bWarnedAboutNotAttached = False
        
        # initialize
        self.name = 'phidgets_interfacekit'
        rospy.init_node(self.name, anonymous=False)
        self.nodename = rospy.get_name()
        self.namespace = rospy.get_namespace()
        
        # Load the parameters.
        self.params = rospy.get_param('%s' % self.nodename.rstrip('/'), {})
        self.defaults = {'serial':0}
        SetDict().set_dict_with_preserve(self.params, self.defaults)
        rospy.set_param('%s' % self.nodename.rstrip('/'), self.params)
        

        # Connect to the Phidget.
        self.interfacekit = Phidgets.Devices.InterfaceKit.InterfaceKit()
        if (self.params['serial']==0):
            self.interfacekit.openPhidget()
        else:
            self.interfacekit.openPhidget(self.params['serial'])
            
        self.interfacekit.setOnAttachHandler(self.attach_callback)
        self.interfacekit.setOnDetachHandler(self.detach_callback)

        # Messages & Services.
        self.subCommand  = rospy.Subscriber('%s/command' % self.nodename.rstrip('/'), String, self.command_callback)
        
        self.service = rospy.Service('%s/get_ai' % self.namespace.rstrip('/'), SrvPhidgetsInterfaceKitGetAI, self.get_ai_callback)
        self.service = rospy.Service('%s/get_di' % self.namespace.rstrip('/'), SrvPhidgetsInterfaceKitGetDI, self.get_di_callback)
        self.service = rospy.Service('%s/set_do' % self.namespace.rstrip('/'), SrvPhidgetsInterfaceKitSetDO, self.set_do_callback)
        rospy.sleep(1) # Allow time to connect publishers & subscribers.


        self.bInitialized = True
        
        
    def attach_callback(self, phidget):
        # Hardware description
        self.nChannelsAI = self.interfacekit.getSensorCount()
        self.nChannelsDI = self.interfacekit.getInputCount()
        self.nChannelsDO = self.interfacekit.getOutputCount()
        self.maxVoltageAI = 5.0
        self.maxADC = 1000
        
        self.phidgetserial = self.interfacekit.getSerialNum()
        self.phidgetname = self.interfacekit.getDeviceName()
        
        rospy.sleep(1) # Wait so that other nodes can display their banner first.
        rospy.logwarn('%s - %s Attached: serial=%s' % (self.namespace, self.phidgetname, self.phidgetserial))
        self.bAttached = True
        self.bWarnedAboutNotAttached = False
        

    def detach_callback(self, phidget):
        rospy.logwarn ('%s - %s Detached: serial=%s' % (self.namespace, self.phidgetname, self.phidgetserial))
        self.bAttached = False
        self.bWarnedAboutNotAttached = False


    # get_ai_callback()
    # Read the analogin voltages, then publish and return them.
    #
    def get_ai_callback(self, req):
        if (self.bInitialized) and (self.bAttached):
            # Read the requested channels.
            voltages = [0.0]*len(req.channels)
            i = 0
            for ch in req.channels:
                try:
                    voltages[i] = self.interfacekit.getSensorValue(ch) * self.maxVoltageAI / self.maxADC
                except Phidgets.PhidgetException, e:
                    rospy.logwarn('%s - PhidgetException: %s' % (self.namespace, e))
                    
                i += 1
                
            rv = SrvPhidgetsInterfaceKitGetAIResponse(voltages)
        else:
            rv = SrvPhidgetsInterfaceKitGetAIResponse()
            if (not self.bWarnedAboutNotAttached):
                rospy.logwarn('%s - PhidgetsInterfaceKit is not attached.' % self.namespace)
                self.bWarnedAboutNotAttached = True

            
        return rv
    
    
    # get_di_callback()
    # Read the digitalin values, then publish and return them.
    #
    def get_di_callback(self, req):
        if (self.bInitialized) and (self.bAttached):
            # Read the requested channels.
            values = [0.0]*len(req.channels)
            i = 0
            for ch in req.channels:
                try:
                    values[i] = self.interfacekit.getInputState(ch)
                except Phidgets.PhidgetException, e:
                    rospy.logwarn('%s - PhidgetException: %s' % (self.namespace, e))

                i += 1
                
            rv = SrvPhidgetsInterfaceKitGetDIResponse(values)
        else:
            rv = SrvPhidgetsInterfaceKitGetDIResponse()
            if (not self.bWarnedAboutNotAttached):
                rospy.logwarn('%s - PhidgetsInterfaceKit is not attached.' % self.namespace)
                self.bWarnedAboutNotAttached = True
            
        return rv
    
    
    def set_do_callback(self, req):
        if (self.bInitialized) and (self.bAttached):
            if (len(req.channels)==len(req.values)):
                # Set the requested channels.
                for i in range(len(req.channels)):
                    try:
                        self.interfacekit.setOutputState(req.channels[i], req.values[i])
                    except Phidgets.PhidgetException, e:
                        rospy.logwarn('%s - PhidgetException: %s' % (self.namespace, e))
            else:
                rospy.logwarn('%s - PhidgetsInterfaceKit: You must specify the same number of digital output values as channels.' % self.namespace)
                
        else:
            if (not self.bWarnedAboutNotAttached):
                rospy.logwarn('%s - PhidgetsInterfaceKit is not attached.' % self.namespace)
                self.bWarnedAboutNotAttached = True
            
        return SrvPhidgetsInterfaceKitSetDOResponse()
    
    
    # command_callback()
    # Handle messages sent to the command topic.
    #
    def command_callback(self, msg):
        if (msg.data == 'exit'):
            rospy.signal_shutdown('User requested exit.')


        elif (msg.data == 'help'):
            rospy.logwarn('')
            rospy.logwarn('Provides ROS access to the digital and analog i/o of the PhidgetsInterfaceKit.')
            rospy.logwarn('Three service calls:')
            rospy.logwarn('  get_ai(channels):         Read the analog input channels, publish the voltages')
            rospy.logwarn('                            to the ROS topic ''ai'', and return the results.')
            rospy.logwarn('  get_di(channels):         Read the digital input channels, publish the voltages')
            rospy.logwarn('                            to the ROS topic ''di'', and return the results.')
            rospy.logwarn('  set_do(channels,values):  Set the digital output channels to the given values.')
            rospy.logwarn('')
            rospy.logwarn('The %s/command topic accepts the following string commands:' % self.namespace.rstrip('/'))
            rospy.logwarn('    help     This message.')
            rospy.logwarn('    exit     Exit the program.')
            rospy.logwarn('')
            rospy.logwarn('You can send the above commands at the shell prompt via:')
            rospy.logwarn('    rostopic pub -1 %s/command std_msgs/String help' % self.namespace.rstrip('/'))
            rospy.logwarn('    rostopic pub -1 %s/command std_msgs/String exit' % self.namespace.rstrip('/'))
            rospy.logwarn('')
            rospy.logwarn('')
            rospy.logwarn('InterfaceKit analog-in and digital-in/out function calls are ')
            rospy.logwarn('provided via ROS services, and may be called from the command-line:')
            rospy.logwarn('    rosservice call %s/get_ai [0,1,2,3,4,5,6,7]' % self.namespace.rstrip('/'))
            rospy.logwarn('    rosservice call %s/get_di [0,1,2,3,4,5,6,7]' % self.namespace.rstrip('/'))
            rospy.logwarn('    rosservice call %s/set_do [0,1,2,3,4,5,6,7] [0,1,1,0,1,0,1,0]' % self.namespace.rstrip('/'))
            rospy.logwarn('')
            rospy.logwarn('')
                    

    def run(self):
        rospy.spin()


if __name__ == '__main__':

    main = PhidgetsInterfaceKit()
    main.run()

