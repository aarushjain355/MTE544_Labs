import numpy as np


from pid import PID_ctrl
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3

class controller:
    
    
    # Default gains of the controller for linear and angular motions
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        # TODO Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller
        self.PID_linear=PID_ctrl(P, klp, klv, kli, filename_="linear.csv")
        self.PID_angular=PID_ctrl(P, kap, kav, kai, filename_="angular.csv")

        self.USE_REAL_ROBOT=False

        # linear and angular velocity limits for the real and simulated robot
        self.real_robot_linear_vel_limit=0.31
        self.real_robot_angular_vel_limit=1.9
        self.simulated_robot_linear_vel_limit=0.26
        self.simulated_robot_angular_vel_limit=1.82

    def vel_request(self, pose, goal, status):
        
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)


        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)
        
        # TODO Part 4: Add saturation limits for the robot linear and angular velocity

        if self.USE_REAL_ROBOT:
            linear_limit = self.real_robot_linear_vel_limit
            angular_limit = self.real_robot_angular_vel_limit
        else:
            linear_limit = self.simulated_robot_linear_vel_limit
            angular_limit = self.simulated_robot_angular_vel_limit

        linear_vel = max(-linear_limit, min(linear_vel, linear_limit))
        angular_vel = max(-angular_limit, min(angular_vel, angular_limit))
        
        return linear_vel, angular_vel

class trajectoryController(controller):

    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        super().__init__(klp, klv, kli, kap, kav, kai)
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals)
        
        finalGoal=listGoals[-1]
        
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)

        
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        # TODO Part 5: Add saturation limits for the robot linear and angular velocity

        if self.USE_REAL_ROBOT:
            linear_limit = self.real_robot_linear_vel_limit
            angular_limit = self.real_robot_angular_vel_limit
        else:
            linear_limit = self.simulated_robot_linear_vel_limit
            angular_limit = self.simulated_robot_angular_vel_limit

        linear_vel = max(-linear_limit, min(linear_vel, linear_limit))
        angular_vel = max(-angular_limit, min(angular_vel, angular_limit))
        
        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]
