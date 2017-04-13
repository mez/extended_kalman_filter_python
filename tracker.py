import numpy as np
from math import pow
from utils import SensorType, polar2cart
from ekf import ExtendedKalmanFilter


class Tracker:
    '''
    The Tracker class is created everytime we detect a new object during our analysis from either
    the LIDAR point cloud or the RADAR doppler. It contains the entire state of the tracked object.
    '''
    def __init__(self):
        #ideally we'd be given this ID from the measurement_packets coming in.
        #However, I currently don't know how multiple objects are Identified.
        self.id  = None

        #this is where the magic happens.
        self.__ekf = ExtendedKalmanFilter()
        self.__is_initialized = False
        self.__previous_timestamp = 0.


    def process_measurement(measurement_packet):
        # if lidar and x,y are zero then I set them to small values.
        if (measurement_packet.sensor_type == SensorType.LIDAR &&
           (measurement_packet.x_measured+measurement_packet.y_measured) == 0):
           measurement_packet.x_measured = 1e-4
           measurement_packet.y_measured = 1e-4

        # if this is first measurement_packet, then setup state vector.
        if not self.__is_initialized:
            self.__previous_timestamp = measurement_packet.timestamp

            if measurement_packet.sensor_type == SensorType.LIDAR:
                self.X = np.array([measurement_packet.x_measured,measurement_packet.y_measured,0,0])
            elif measurement_packet.sensor_type == SensorType.RADAR:
                #we have the polar space measurements; we need to transform to cart space.
                x,y = polar2cart()
                self.X = np.array([x,y,0,0])

            self.__is_initialized = True

        #1st we calculate how much time has passed since our last measurement_packet in seconds
        dt =( measurement_packet.timestamp - self.__previous_timestamp) / 1000000.0
        self.__previous_timestamp = measurement_packet.timestamp

        #2nd set new F and Q using new dt
        self.__ekf.set_F_and_Q(dt)

        #3rd update Q with dt
