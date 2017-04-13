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
        self.noise_ax = 5;
	    self.noise_ay = 5;

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
            dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
        '''

    def predict(self):
        NotImplementedError

    def updateLidar(self):
        NotImplementedError

    def updateRadar(self):
        NotImplementedError
