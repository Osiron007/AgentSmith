import numpy as np
from random import randint
from OU import OU

OU = OU()       #Ornstein-Uhlenbeck Process

class exploration_noise_generator(object):

    def __init__(self,static_params, OU_mu, OU_theta, OU_sigma, use_ANN):
        print("Init exploration noise generator")

        #defines if different working points should be generated or not
        self.static = static_params
        #if static_params = true use this parameter
        self.mu = OU_mu           #gleichgewichtswert
        self. theta = OU_theta    #anziehungskraft
        self.sigma = OU_sigma     #diffusionsterm

        self.working_point_left = 0.0
        self.working_point_right = 0.0

        #NoiseV2
        self.noise_cnt = 0



    def get_exploration_noiseV1(self, current_value, wheel_side):
        #print("Get noise")

        if(wheel_side == 1):
            self.mu = self.working_point_left
        if(wheel_side == 2):
            self.mu = self.working_point_right


        return OU.function(current_value, self.mu, self.theta, self.sigma)

    def generate_working_points(self):
        # generate working point
        # avaliable working points: 1, 0.75, 0.5, 0.25, 0.0, -0.25, -0.5, -0.75, -1

        working_point_category = randint(0, 8)
        # print("RND: " + str(working_point_category))
        if working_point_category == 0:
            working_point = 0.0
        if working_point_category > 0 and working_point_category < 5:
            working_point = working_point_category * 0.25
        if working_point_category >= 5:
            working_point_category = working_point_category - 4
            working_point = -working_point_category * 0.25

        self.working_point_left = working_point

        working_point_category = randint(0, 8)
        # print("RND: " + str(working_point_category))
        if working_point_category == 0:
            working_point = 0.0
        if working_point_category > 0 and working_point_category < 5:
            working_point = working_point_category * 0.25
        if working_point_category >= 5:
            working_point_category = working_point_category - 4
            working_point = -working_point_category * 0.25

        self.working_point_right = working_point

        print("Current Working Points: left: " + str(self.working_point_left))
        print("Current Working Points: right: " + str(self.working_point_right))

    def get_exploration_noiseV2(self, current_value, wheel_side):

        # always 6 cycles without noise and 12 cycles with noise
        # to ensure exploration without blocking
        if self.noise_cnt > 6:
            # reset cnt
            if self.noise_cnt > 24:
                self.noise_cnt = 0
            else:
                self.noise_cnt = self.noise_cnt + 1
            # generate noise
            if (current_value > 0):
                noise = randint(-180, -1)
            else:
                noise = randint(1, 180)

            noise = np.float(np.float(noise) / 100)
            # print("############")
            # print("Noise: " + str(noise))
            return noise  # OU.function(current_value, self.mu, self.theta, self.sigma)
        else:
            self.noise_cnt = self.noise_cnt + 1
            return 0.0
