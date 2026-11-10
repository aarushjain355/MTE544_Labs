

import numpy as np



# TODO Part 3: Comment the code explaining each part
class kalman_filter:
    
    # TODO Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        
        self.P= P # State covariance matrix
        self.Q= Q # State noise covariance matrix
        self.R= R # Measurement noise covariance matrix
        self.x= x # Init state 
        self.dt = dt # Small time step
        
    # TODO Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):

        # Calculate motion model and measurement model jacobians at current time step
        self.A = self.jacobian_A()
        self.C = self.jacobian_H()

        # Predict state based on motion model at current time step
        self.motion_model()

        # Predict state covariance based on motion model jacobian and state noise covariances
        # at current time step 
        self.P= np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

    # TODO Part 3: Calculate the Kalman gain and correct your estimates
    def update(self, z):

        # Compute kalman gain based on relative uncertainty between state and measurement model
        # at current time step 
        S = np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        kalman_gain = np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))

        # Compute innovation based on measurement and expected measurement from measurement model
        # at current time step
        y_bar = z - self.measurement_model()

        # Update state based on innovation weighted by kalman gain
        # at current time step 
        self.x = self.x + np.dot(kalman_gain, y_bar)

        # Update state covariance accounting for measurement incorporation
        # at current time step
        KG_C = np.dot(kalman_gain, self.C)
        self.P = np.dot((np.eye(KG_C.shape[0], KG_C.shape[1]) - KG_C), self.P)
        
    
    # TODO Part 3: Implement here the measurement model
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v, # v
            w, # w
            vdot, # ax
            v * w, # ay
        ])
        
    # TODO Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        self.x = np.array([
            x + v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])
        


    
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt

        dg1 = -v * dt * np.sin(th)
        dg2 =  v * dt * np.cos(th)
        dg3 = dt * np.cos(th)
        dg4 = dt * np.sin(th)

        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0,              dg1, 0,          dg3,  0],
            [0, 1,              dg2, 0,          dg4,  0],
            [0, 0,                1, dt,           0,  0],
            [0, 0,                0, 1,            0,  0],
            [0, 0,                0, 0,            1,  dt],
            [0, 0,                0, 0,            0,  1 ]
        ])
    
    
    # TODO Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0], # v
            [0,0,0  , 1, 0, 0], # w
            [0,0,0  , 0, 0, 1], # ax
            [0,0,0  , v, w, 0], # ay
        ])
        
    # TODO Part 3: return the states here    
    def get_states(self):
        return self.x
