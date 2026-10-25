import math

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

PARABOLA=0; SIGMOID=1

class planner:
    def __init__(self, type_):

        self.type=type_
        self.num_points=50 # number of points in the trajectory
        self.trajectory_type=PARABOLA # default trajectory planner

        # starting and ending points for parabolic trajectory
        self.parabola_x_start = 0.0
        self.parabola_x_end = 1.5

        # starting and ending points for sigmoid trajectory
        self.sigmoid_x_start = 0.0
        self.sigmoid_x_end = 2.5

    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        if self.trajectory_type == PARABOLA:
            return self.parabola_trajectory(self.parabola_x_start, self.parabola_x_end)
        elif self.trajectory_type == SIGMOID:
            return self.sigmoid_trajectory(self.sigmoid_x_start, self.sigmoid_x_end)

    def parabola_trajectory(self, x_start, x_end):
        """
        Generate a parabolic trajectory based on the equation y = ax^2 + bx + c.
        """
        trajectory = []
        for i in range(self.num_points):
            x = x_start + (x_end - x_start) * i / (self.num_points - 1)
            y = x**2
            trajectory.append([x, y])
        return trajectory


    def sigmoid_trajectory(self, x_start, x_end):
        """
        Generate a sigmoid trajectory based on the equation y = 2 / (1 + exp(-2 * x)) - 1.
        """
        trajectory = []
        for i in range(self.num_points):
            x = x_start + (x_end - x_start) * i / (self.num_points - 1)
            y = 2.0 / (1.0 + math.exp(-2.0 * x)) - 1.0
            trajectory.append([x, y])
        return trajectory