#!/usr/bin/env python
#
#   gripper
#
#   Class to handle communication with the gripper
#
#   Subscribers:    none
#   Publishers:     none
#
#   Services:       none
#

import socket

class Gripper(object):
    '''
    Gripper class to handle sending messages to control the gripper via raspberry pi
    '''
    def __init__(self):
        '''
        Initialization. No arguments
        '''
        self.UDP_PORT = 10000 # Hard coded to use port 10000
        self.UDP_IP = '10.10.10.99' # Static IP of RasPi is 10.10.10.99

        try: # Attempt to create a UDP socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except:
            self.socket = None

    def on(self):
        '''
        Sends the command to turn the gripper on.
        '''
        if not (self.socket == None):
            self.socket.sendto('1', (self.UDP_IP, self.UDP_PORT))

    def off(self):
        '''
        Sends the command to turn the gripper off,
        '''
        if not (self.socket == None):
            self.socket.sendto('0', (self.UDP_IP, self.UDP_PORT))