#!/usr/bin/env python
#
#   states.py

from enum import Enum


class HockbotStates(Enum):
    '''
    Class to store all constants used for the entire robot
    '''
    HOCKBOT_STATE_IDLE = 0
    HOCKBOT_STATE_DEFENSE = 1
    HOCKBOT_STATE_SCANNING = 2
    HOCKBOT_STATE_SHOOTING = 3
    HOCKBOT_STATE_GRABFROMTABLE = 4
    HOCKBOT_STATE_GRABFROMGOAL = 5
    HOCKBOT_STATE_PUCKSETUP = 6


class SCARAStates(Enum):
    SCARA_STATE_IDLE = 0
    SCARA_STATE_TRIANGLEDEFENSE = 1
    SCARA_STATE_SCANNING = 2
    SCARA_STATE_SHOOTING = 3


class PickerStates(Enum):
    PICKER_STATE_IDLE = 0
    PICKER_STATE_GRABFROMTABLE = 4
    PICKER_STATE_GRABFROMGOAL = 5
    PICKER_STATE_PUCKSETUP = 6
