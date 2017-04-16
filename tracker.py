from utils import SensorType, polar_2_cart
from ekf import ExtendedKalmanFilter
import numpy as np

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

    @property
    def state(self):
        return self.__ekf.current_estimate[0]

    def process_measurement(self,measurement_packet):
        # if this is first measurement_packet, then setup state vector.
        if not self.__is_initialized:
            x, y, vx, vy = 0,0,0,0
            if measurement_packet.sensor_type == SensorType.LIDAR:
                x, y, vx, vy = measurement_packet.x_measured,measurement_packet.y_measured,0,0

            elif measurement_packet.sensor_type == SensorType.RADAR:
                #we have the polar space measurements; we need to transform to cart space.
                x,y, vx, vy = polar_2_cart(measurement_packet.rho_measured,
                                 measurement_packet.phi_measured,
                                 measurement_packet.rhodot_measured)

            if abs(x+y) <= 1e-4:
                x = 1e-4
                y = 1e-4

            self.__ekf.init_state_vector(x,y, vx, vy)
            self.__previous_timestamp = measurement_packet.timestamp
            self.__is_initialized = True
            return


        #1st we calculate how much time has passed since our last measurement_packet in seconds
        dt =( measurement_packet.timestamp - self.__previous_timestamp) / 1000000.0
        self.__previous_timestamp = measurement_packet.timestamp

        #2nd set new F and Q using new dt
        self.__ekf.recompute_F_and_Q(dt)

        #3rd make a prediction
        self.__ekf.predict()

        #4th update prediction
        if measurement_packet.sensor_type == SensorType.LIDAR:
            self.__ekf.updateLidar(measurement_packet)
        elif measurement_packet.sensor_type == SensorType.RADAR:
            self.__ekf.updateRadar(measurement_packet)
