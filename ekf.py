import numpy as np
import numpy.matlib
from math import pi, sqrt

from utils import cart_2_polar, state_vector_to_scalars

class ExtendedKalmanFilter:
    def __init__(self):
        '''
        Each object being tracked will result in the creation of a new ExtendedKalmanFilter instance.

        TODO: consider making these just class methods; that way we don't have many instances and instead
        each tracker will just call to these methods with the matrices to update.
        '''

        #do this once do we don't keep redoing in update step
        self.__xI = np.matlib.identity(4)

        self.__x = None
        self.__F = None
        self.__Q = None

        self.__P = np.matrix([[1,0,0,0],
                              [0,1,0,0],
                              [0,0,1000,0],
                              [0,0,0,1000]])

        self.__HL = np.matrix([[1,0,0,0],
                               [0,1,0,0]])

        self.__HR = np.matlib.zeros((3,4))

        self.__RL = np.matrix([[0.0225,0],
                               [0,0.0225]])

        self.__RR = np.matrix([[0.09,0,0],
                               [0,0.0009,0],
                               [0,0,0.09]])

        #we can adjust these to get better accuracy
        self.__noise_ax = 9
        self.__noise_ay = 9

    @property
    def current_estimate(self):
        return (self.__x, self.__P)

    def init_state_vector(self, x,y, vx, vy):
        self.__x = np.matrix([[x,y,vx,vy]]).T

    def recompute_F_and_Q(self, dt):
        '''
        updates the motion model and process covar based on delta time from last measurement.
        '''

        #set F
        self.__F = np.matrix([[1,0,dt,0],
                              [0,1,0,dt],
                              [0,0,1,0],
                              [0,0,0,1]])
        #set Q
        dt2 = dt**2
        dt3 = dt**3
        dt4 = dt**4

        e11 = dt4 * self.__noise_ax / 4
        e13 = dt3 * self.__noise_ax / 2
        e22 = dt4 * self.__noise_ay / 4
        e24 = dt3 * self.__noise_ay /  2
        e31 = dt3 * self.__noise_ax / 2
        e33 = dt2 * self.__noise_ax
        e42 = dt3 * self.__noise_ay / 2
        e44 = dt2 * self.__noise_ay

        self.__Q = np.matrix([[e11, 0, e13, 0],
                              [0, e22, 0, e24],
                              [e31, 0, e33, 0],
                              [0, e42, 0, e44]])

    def recompute_HR(self):
        '''
        calculate_jacobian of the current state.
        '''
        px,py,vx,vy = state_vector_to_scalars(self.__x)

        pxpy_squared = px**2+py**2
        pxpy_squared_sqrt = sqrt(pxpy_squared)
        pxpy_cubed = (pxpy_squared*pxpy_squared_sqrt)

        if pxpy_squared < 1e-4:
            self.__HR = np.matlib.zeros((3,4))
            return

        e11 = px/pxpy_squared_sqrt
        e12 = py/pxpy_squared_sqrt
        e21 = -py/pxpy_squared
        e22 = px/pxpy_squared
        e31 = py*(vx*py - vy*px)/pxpy_cubed
        e32 = px*(px*vy - py*vx)/pxpy_cubed

        self.__HR = np.matrix([[e11, e12, 0, 0],
                               [e21, e22, 0, 0],
                               [e31, e32, e11, e12]])

    def predict(self):
        '''
        This is a projection step. we predict into the future.
        '''
        self.__x = self.__F * self.__x
        self.__P = (self.__F * self.__P * self.__F.T) + self.__Q

    def updateLidar(self,measurement_packet):
        '''
        This is the projection correction; after we predict we use the sensor data
        and use the kalman gain to figure out how much of the correction we need.
        '''

        #this is the error of our prediction to the sensor readings
        y = measurement_packet.z - self.__HL*self.__x

        #pre compute for the kalman gain K
        PHLt = self.__P * self.__HL.T
        S = self.__HL * PHLt + self.__RL
        K = PHLt*S.I

        #now we update our prediction using the error and kalman gain.
        self.__x += K*y
        self.__P = (self.__xI - K*self.__HL) * self.__P

    def updateRadar(self,measurement_packet):
        '''
        This is the projection correction; after we predict we use the sensor data
        and use the kalman gain to figure out how much of the correction we need.

        This is a special case as we will need a Jocabian matrix to have a linear
        approximation of the transformation function h(x)
        '''

        y = measurement_packet.z - cart_2_polar(self.__x)
        #make sure the phi in y is -pi <= phi <= pi
        while (y[1] > pi): y[1] -= 2.*pi
        while (y[1] < -pi): y[1] += 2.*pi

        #recompute Jacobian
        self.recompute_HR()

        #pre compute for the kalman gain K
        #TODO: this code is not DRY should refactor here.
        S = self.__HR * self.__P * self.__HR.T + self.__RR
        K = self.__P*self.__HR.T*S.I

        #now we update our prediction using the error and kalman gain.
        self.__x += K*y
        self.__P = (self.__xI - K*self.__HR) * self.__P
