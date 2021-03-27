import numpy as np

from libs.normalise_angle import normalise_angle

class PathTracker:

    def __init__(self, control_gain, softening_gain, steering_limits, centreofgravity_to_frontaxle, target_vel, x, y, yaw, path_x, path_y, path_yaw):
        
        # Class variables to use whenever within the class when necessary
        self.k = control_gain
        self.ksoft = softening_gain
        self.max_steer = steering_limits
        self.cg2frontaxle = centreofgravity_to_frontaxle

        self.x = x
        self.y = y
        self.yaw = yaw
        self.target_vel = target_vel

        self.cx = path_x
        self.cy = path_y
        self.cyaw = path_yaw

        self.target_idx = None

    def target_index_calculator(self):  

        ''' Calculates the target index and each corresponding error '''

        # Calculate position of the front axle
        fx = self.x + self.cg2frontaxle * np.cos(self.yaw)
        fy = self.y + self.cg2frontaxle * np.sin(self.yaw)

        dx = [fx - icx for icx in self.cx] # Find the x-axis of the front axle relative to the path
        dy = [fy - icy for icy in self.cy] # Find the y-axis of the front axle relative to the path

        d = np.hypot(dx, dy) # Find the distance from the front axle to the path
        target_idx = np.argmin(d) # Find the shortest distance in the array

        # Cross track error, project RMS error onto the front axle vector
        front_axle_vec = [-np.cos(self.yaw + np.pi/2), -np.sin(self.yaw + np.pi/2)]
        crosstrack_error = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        # Heading error
        heading_term = normalise_angle(self.cyaw[target_idx] - self.yaw)

        self.target_idx = target_idx

        return crosstrack_error, heading_term

    # Stanley controller determines the appropriate steering angle
    def stanley_control(self):

        crosstrack_error, heading_term = self.target_index_calculator()
        crosstrack_term = np.arctan2((self.k * crosstrack_error), (self.ksoft + self.target_vel))
        
        sigma_t = crosstrack_term + heading_term

        # Constrains steering angle to the vehicle limits
        if sigma_t >= self.max_steer:
            sigma_t = self.max_steer

        elif sigma_t <= -self.max_steer:
            sigma_t = -self.max_steer

        return self.target_vel, sigma_t, crosstrack_term

def main():

    print("This script is not meant to be executable, and should be used as a library.")

if __name__ == "__main__":
    main()