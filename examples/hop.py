# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

"""
Connects to the first Crazyflie found, does a little jump.
"""

import time
import sys
import logging
import traceback

sys.path.append("../lib")
import cflib  # noqa
from cflib.crazyflie import Crazyflie  # noqa

logging.basicConfig(level=logging.ERROR)


class MotorRampExample:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)

        print("Connecting to %s" % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        try:
            self._stunt()
        except Exception as ex:
            print("Stunt raised an exception. Shutting down.")
            traceback.print_exc()
        finally:
            # Make sure that the last packet leaves before the link is closed
            # since the message queue is not flushed before closing
            self._cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)
            self._cf.close_link()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print("Connection to %s failed: %s" % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print("Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print("Disconnected from %s" % link_uri)

    def _stunt(self):
        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        thrust_hover = 38000

        # Launch
        self._cf.commander.send_setpoint(roll=0, pitch=0, yaw=0, thrust=65000)
        time.sleep(0.25)

        # Hover
        self._cf.commander.send_setpoint(roll=0, pitch=0, yaw=0, thrust=thrust_hover)
        time.sleep(1.0)

        # Back up.
        self._cf.commander.send_setpoint(roll=0, pitch=-30, yaw=0, thrust=thrust_hover)
        time.sleep(.5)

        # Hover
        self._cf.commander.send_setpoint(roll=0, pitch=0, yaw=0, thrust=thrust_hover)
        time.sleep(1.0)

        # Forwards.
        self._cf.commander.send_setpoint(roll=0, pitch=30, yaw=0, thrust=thrust_hover)
        time.sleep(.5)

        # Land.
        self._drop()

    def _drop(self):
        thrust_start = 35000
        thrust_end = 29000
        duration = 1.5
        tstep = 0.02
        for t in frange(0, duration, tstep):
            thrust = int(self._tween(thrust_start, thrust_end, t / duration))
            self._cf.commander.send_setpoint(roll=0, pitch=0, yaw=0, thrust=thrust)
            time.sleep(tstep)

    def _tween(self, start, end, x):
        """A value goes from start to end and x is in [0,1]"""
        x = float(x)
        return start + (end - start) * x

def frange(start, stop, step):
    assert start < stop
    assert step > 0

    acc = start
    while acc < stop:
        yield acc
        acc += step

def scan_for_crazyflies():
    # Scan for Crazyflies and use the first one found
    print("Scanning interfaces for Crazyflies...")
    available = cflib.crtp.scan_interfaces()
    print("Crazyflies found:")
    for i in available:
        print(i[0])

    if len(available) > 0:
        return available[0][0]
    else:
        print("No Crazyflies found")
        return None

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # scan_for_crazyflies()

    radio = "radio://0/80/250K"
    # le = MotorRampExample(available[0][0])
    le = MotorRampExample(radio)
