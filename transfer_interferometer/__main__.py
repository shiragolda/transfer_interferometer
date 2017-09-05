"""Logs the error signals of the transfer interferometer."""

import time
import random
import zmq
import datetime
import sys
import numpy as np
from scipy.interpolate  import interp1d
import os
import serial
import threading
import argparse



parser = argparse.ArgumentParser(description='Reads and publishes the cavity'
                                 'temperature on a zeromq socket.\n')
parser.add_argument("-s", "--serialport", type=str,
                    help='Serial port to use to communicate with the'
                         'wavemeter. e.g /dev/ttyUSB0 for linux, '
                         'COM10 for windows',
                    default='COM12')
parser.add_argument("-p", "--publishport", type=int,
                    help='zeromq port to use to broadcast the wavemeter'
                         'reading.',
                    default=5552)
parser.add_argument("-t", "--topic", type=str,
                    help='topic to use when broadcasting. e.g wa1500-lab1',
                    default='interferometer')
args = parser.parse_args()

class zmq_pub_dict:
    """Publishes a python dictionary on a port with a given topic."""

    def __init__(self, port, topic):
        zmq_context = zmq.Context()
        self.topic = topic
        self.pub_socket = zmq_context.socket(zmq.PUB)

        self.pub_socket.bind("tcp://*:%s" % port)
        print('Broadcasting on port {0} with topic {1}'.format(port,
                                                               topic))

    def send(self, data_dict):
        timestamp = time.time()
        send_string = "%s %f %s" % (self.topic, timestamp, repr(data_dict))
        print(send_string)
        self.pub_socket.send(send_string)

    def close(self):
        self.pub_socket.close()


publisher = zmq_pub_dict(args.publishport, "interferometer_error_signals")

ser = serial.Serial()
ser.port = args.serialport
ser.baudrate = 115200
ser.timeout = 2
ser.setDTR(False)
ser.open()

kbd_input = ''
new_input = True

def commandListener():
    global kbd_input, new_input
    kbd_input = raw_input()
    new_input = True


def get_error_signal():
    s = ser.readline()
    try:
        error1,correction1,error2,correction2,error3,correction3 = s.split(',')
        error1 = float(error1)
        error2 = float(error2)
        error3 = float(error3)
        correction1 = float(correction1)
        correction2 = float(correction2)
        correction3 = float(correction3)
        msg = 'ok'
    except:
        error1 = 0
        error2 = 0
        error3 = 0
        correction1 = 0
        correction2 = 0
        correction3 = 0
        msg = s

    return error1,correction1,error2,correction2,error3,correction3,msg


done = False
while not done:
    try:
        error1,correction1,error2,correction2,error3,correction3,msg = get_error_signal();

        if msg=='ok':
            data_dict = {'interferometer_error': error1, 'interferometer_correction': correction1, '423_laser_error': error2, '423_laser_correction': correction2,'453_laser_error': error3, '453_laser_correction': correction3}

            dt = str(time.time())
            publisher.send(data_dict)

        elif msg!='':
            print msg


        if new_input:
            ser.write(kbd_input)
            new_input = False
            listener = threading.Thread(target=commandListener)
            listener.start()


    except KeyboardInterrupt:
        ser.close()
        done = True
        publisher.close()
publisher.close()

