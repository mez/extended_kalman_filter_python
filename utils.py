'''
utils provides helper classes and methods.
'''

from enum import Enum

def polar_2_cart(ro, phi, ro_dot):
    '''
    ro: range
    phi: bearing
    ro_dot: range rate

    Takes the polar coord based radar reading and convert to cart coord x,y
    return (x,y)
    '''
    NotImplementedError

def calculate_rmse(estimations, ground_truth):
    '''
    Root Mean Squared Error.
    '''
    NotImplementedError

def calculate_jacobian(state):
    '''
    Creates a Jacobian matrix from the state vector. This is a polynomial approximation of the
    funtion that maps the state vector to the polar coordinates.
    '''
    NotImplementedError


class SensorType(Enum):
    '''
    Enum types for the sensors. Future sensors would be added here.
    '''
    LIDAR = 'L'
    RADAR = 'R'


class MeasurementPacket:
    '''
    Defines a measurement datapoint for multiple sensor types.
    '''
    def __init__(self, packet):
        self.sensor_type = SensorType.LIDAR if packet[0] == 'L' else SensorType.RADAR

        if self.sensor_type == SensorType.LIDAR:
            self.setup_lidar(packet)
        elif self.sensor_type == SensorType.RADAR:
            self.setup_radar(packet)

    def setup_radar(self, packet):
        self.rho_measured       = packet[1]
        self.phi_measured       = packet[2]
        self.rhodot_measured    = packet[3]
        self.timestamp          = packet[4]
        self.x_groundtruth      = packet[5]
        self.y_groundtruth      = packet[6]
        self.vx_groundtruth     = packet[7]
        self.vy_groundtruth     = packet[8]

    def setup_lidar(self, packet):
        self.x_measured         = packet[1]
        self.y_measured         = packet[2]
        self.timestamp          = packet[3]
        self.x_groundtruth      = packet[4]
        self.y_groundtruth      = packet[5]
        self.vx_groundtruth     = packet[6]
        self.vy_groundtruth     = packet[7]

    def __str__(self):
        if self.sensor_type == SensorType.LIDAR:
            return "LIDAR (timestamp: {:>8}) \n MEASUREMENT [{:>4} {:>4}] \n  GROUND TRUTH [{:>4} {:>4} {:>4} {:>4}]".format(
                    self.timestamp,

                    self.x_measured,
                    self.y_measured ,

                    self.x_groundtruth ,
                    self.y_groundtruth,
                    self.vx_groundtruth,
                    self.vy_groundtruth)

        elif self.sensor_type == SensorType.RADAR:
            return "RADAR (timestamp: {:>8}) \n MEASUREMENT [{:>4} <> {:>4} <> {:>4}] \n GROUND TRUTH [{:>4} <> {:>4} <> {:>4} <> {:>4}]".format(
                    self.timestamp    ,

                    self.rho_measured,
                    self.phi_measured ,
                    self.rhodot_measured,

                    self.x_groundtruth,
                    self.y_groundtruth ,
                    self.vx_groundtruth,
                    self.vy_groundtruth )
