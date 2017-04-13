import numpy as np
import numpy.matlib
from math import pow

class ExtendedKalmanFilter:
    def __init__(self):
        '''
        Each object being tracked will result in the creation of a new ExtendedKalmanFilter instance.

        TODO: consider making these just class methods; that way we don't have many instances and instead
        each tracker will just call to these methods with the matrices to update.
        '''

        self.X = None
        #do this once do we don't keep redoing in update step
        self.XI = np.matlib.identity(4)

        self.F = np.matrix('1, 0, 1, 0; \
                            0, 1, 0, 1; \
                            0, 0, 1, 0; \
                            0, 0, 0, 1')

        self.P = np.matrix('1, 0, 0,    0; \
                            0, 1, 0,    0; \
                            0, 0, 1000, 0; \
                            0, 0, 0, 1000')

        self.HL = np.matrix('1, 0, 0, 0; \
                             0, 1, 0, 0' )
        self.HR = None

        self.RL = np.matrix('0.0225, 0; \
                             0,      0.0225')
        self.RR = np.matrix('0.09, 0,      0; \
                             0,    0.0009, 0; \
                             0,    0,      0.09')

        self.Q = np.matlib.zeros((4,4))

        #we can adjust these to get better accuracy
        self.noise_ax = 9
        self.noise_ay = 9

    @property
    def current_estimate(self):
        return (self.X, self.P)

    def init_state_vector(x,y):
        self.X = np.array([x,y,0,0]).reshape((4,1))

    def set_F_and_Q(dt):
        '''
        updates the motion model and process covar based on delta time from last measurement.
        '''

        #set F
        self.F[0,2] = dt
        self.F[1,3] = dt

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
        self.Q[0,1] = dt_4/4*self.noise_ax
        self.Q[0,2] = dt_3/2*self.noise_ax

        self.Q[1,1] = dt_4/4*self.noise_ay
        self.Q[1,3] = dt_3/2*self.noise_ay

        self.Q[2,0] = dt_3/2*self.noise_ax
        self.Q[2,2] = dt_2*self.noise_ax

        self.Q[3,1] = dt_3/2*self.noise_ay
        self.Q[3,3] = dt_2*self.noise_ay

    def predict(self):
        '''
        This is a projection step. we predict into the future.
        '''
        self.X = self.F * self.X
        self.P = (self.F * self.P * self.F.T) + self.Q

    def updateLidar(self,measurement_packet):
        '''
        This is the projection correction; after we predict we use the sensor data
        and use the kalman gain to figure out how much of the correction we need.
        '''

        #this is the error of our prediction to the sensor readings
        y = measurement_packet.z - self.HL*self.X
        #pre compute for the kalman gain K
        S = self.HL * self.P * self.HL.T + self.RL
        K = self.P*self.HL.T*S.I

        #now we update our prediction using the error and kalman gain.
        self.X += K*y
        self.P = (self.XI - K*self.HL) * self.P

    def updateRadar(self,measurement_packet):
        '''
        This is the projection correction; after we predict we use the sensor data
        and use the kalman gain to figure out how much of the correction we need.

        This is a special case as we will need a Jocabian matrix to have a linear
        approximation of the transformation function h(x)
        '''

        raise NotImplementedError
