import numpy as np
import numpy.matlib
from math import pow, pi

from utils import cart_2_polar, calculate_jacobian

class ExtendedKalmanFilter:
    def __init__(self):
        '''
        Each object being tracked will result in the creation of a new ExtendedKalmanFilter instance.

        TODO: consider making these just class methods; that way we don't have many instances and instead
        each tracker will just call to these methods with the matrices to update.
        '''

        self.__x = None
        #do this once do we don't keep redoing in update step
        self.__xI = np.matlib.identity(4)

        self.__F = np.matrix('1, 0, 1, 0; \
                            0, 1, 0, 1; \
                            0, 0, 1, 0; \
                            0, 0, 0, 1')

        self.__P = np.matrix('1, 0, 0,    0; \
                            0, 1, 0,    0; \
                            0, 0, 1000, 0; \
                            0, 0, 0, 1000')

        self.__HL = np.matrix('1, 0, 0, 0; \
                             0, 1, 0, 0' )

        self.__HR = np.matlib.zeros((3,4))

        self.__RL = np.matrix('0.0225, 0; \
                             0,      0.0225')
        self.__RR = np.matrix('0.09, 0,      0; \
                             0,    0.0009, 0; \
                             0,    0,      0.09')

        self.__Q = np.matlib.zeros((4,4))

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
        self.__F[0,2] = dt
        self.__F[1,3] = dt

        #set Q
        dt2 = dt**2
        dt3 = dt**3
        dt4 = dt**4

        self.__Q[0,1] = dt4/4*self.__noise_ax
        self.__Q[0,2] = dt3/2*self.__noise_ax

        self.__Q[1,1] = dt4/4*self.__noise_ay
        self.__Q[1,3] = dt3/2*self.__noise_ay

        self.__Q[2,0] = dt3/2*self.__noise_ax
        self.__Q[2,2] = dt2*self.__noise_ax

        self.__Q[3,1] = dt3/2*self.__noise_ay
        self.__Q[3,3] = dt2*self.__noise_ay

    def recompute_HR(self):
        '''
        calculate_jacobian of the current state.
        '''
        self.__HR = calculate_jacobian(self.__x)

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
