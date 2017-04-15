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

        self.__X = None
        #do this once do we don't keep redoing in update step
        self.__XI = np.matlib.identity(4)

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
        self.__noise_ax = 5
        self.__noise_ay = 5

    @property
    def current_estimate(self):
        return (self.__X, self.__P)

    def init_state_vector(self, x,y):
        self.__X = np.array([x,y,0,0]).reshape((4,1))

    def recompute_F_and_Q(self, dt):
        '''
        updates the motion model and process covar based on delta time from last measurement.
        '''

        #set F
        self.__F[0,2] = dt
        self.__F[1,3] = dt

        #set Q
        dt2 = pow(dt, 2)
        dt3 = pow(dt, 3)
        dt4 = pow(dt, 4)

        '''
            For now we will take the process noise as the following matrix.

            dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
        '''

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
        self.__HR = calculate_jacobian(self.__X)

    def predict(self):
        '''
        This is a projection step. we predict into the future.
        '''
        self.__X = self.__F * self.__X
        self.__P = (self.__F * self.__P * self.__F.T) + self.__Q

    def updateLidar(self,measurement_packet):
        '''
        This is the projection correction; after we predict we use the sensor data
        and use the kalman gain to figure out how much of the correction we need.
        '''

        #this is the error of our prediction to the sensor readings
        y = measurement_packet.z - self.__HL*self.__X

        #pre compute for the kalman gain K
        S = self.__HL * self.__P * self.__HL.T + self.__RL
        K = self.__P*self.__HL.T*S.I

        #now we update our prediction using the error and kalman gain.
        self.__X += K*y
        self.__P = (self.__XI - K*self.__HL) * self.__P

    def updateRadar(self,measurement_packet):
        '''
        This is the projection correction; after we predict we use the sensor data
        and use the kalman gain to figure out how much of the correction we need.

        This is a special case as we will need a Jocabian matrix to have a linear
        approximation of the transformation function h(x)
        '''

        y = measurement_packet.z - cart_2_polar(self.__X)
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
        self.__X += K*y
        self.__P = (self.__XI - K*self.__HR) * self.__P
