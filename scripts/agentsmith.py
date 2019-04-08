####################
import sys

from time import *

import numpy as np

###################PARAMETER#######################################

from parameter_handler import parameter_handler as PH

#####################ROS#########################################
import rospy

from ros_environment import ros_environment_handler as REH

from geometry_msgs.msg import Pose as pose_msg_type
from geometry_msgs.msg import PoseStamped as pose_stamped_msg_type

from rl_msgs.msg import Transition

#####################LEARNING#########################################
from reward_calculator import reward_calculation as RC
from exploration_noise import exploration_noise_generatorV1 as noise_gen
from goal_point_generator import goal_point_generator as gpl_gen
from ann import ANN




CONFIG_FILE = 'configuration_1.cfg'


def ST_STARTUP():
    #print("ST_STARTUP")

    ##########################################
    #          parameter_handler             #
    ##########################################
    global params

    ##########################################
    #              ros_handler               #
    ##########################################
    global ros_handler

    ##########################################
    #            noise_generator             #
    ##########################################
    global noise_generator

    ##########################################
    #           reward_calculator            #
    ##########################################
    global reward_calculator

    ##########################################
    #             gpl_generator              #
    ##########################################
    global gpl_generator




    global startup_cnt

    global step

    global goal_pose

    global start_time_episode

    global episode

    global episode_done

    global s_t



    if startup_cnt == 0:
        print("Startup Episode")
        startup_cnt = startup_cnt + 1
        #gpl_generator.use_goal_point_list(gpl_index)

        #obstacle_handler.spawn_obstacles(list_id=1)

        return "ST_STARTUP"
    if startup_cnt == 5:
        # set initial pose
        ros_handler.set_initial_pose()
        # get rnd goal pose
        #goal_pose = reward_calculator.goal_point_generator(False)
        goal_pose = gpl_generator.get_goal_point(params.gpl_use_random_goal, params.gpl_use_random_orientation)
        # set goal pose
        ros_handler.set_goal_pose(goal_pose)
        #reset reward calculation
        reward_calculator.reset_calculation()
        #generate working points for exploration noise
        noise_generator.generate_working_points()

        startup_cnt = startup_cnt + 1
        return "ST_STARTUP"
    if startup_cnt > 15:

        #reset variables
        startup_cnt = 0
        start_time_episode = time()
        step = 0
        episode = episode + 1
        episode_done = False
        collision = False

        print("Startup done")
        return "ST_EXECUTING"

    startup_cnt = startup_cnt + 1
    return "ST_STARTUP"


def ST_EXECUTING():
    #print("ST_EXECUTING")

    ##########################################
    #          parameter_handler             #
    ##########################################
    global params

    ##########################################
    #              ros_handler               #
    ##########################################
    global ros_handler

    ##########################################
    #            action_generator            #
    ##########################################
    global action_generator

    ##########################################
    #            noise_generator             #
    ##########################################
    global noise_generator

    ##########################################
    #           reward_calculator            #
    ##########################################
    global reward_calculator

    ##########################################
    #             gpl_generator              #
    ##########################################
    global gpl_generator


    #episode handling
    global step
    global episode
    global episode_done
    global first_action_in_episode
    global TimeOutOrAway
    global total_reward

    #goal pose
    global goal_pose
    # robot pose
    global robot_current_map_pose_stamped


    #timing
    global start_time_episode

    #Data handling
    global s_t
    global a_t_transition


    params.epsilon -= 1.0 / params.EXPLORE

    if params.epsilon < 0.1:
        params.epsilon = 1.0

    a_t = np.zeros([1, params.action_dim])
    noise_t = np.zeros([1, params.action_dim])

    ####################################################################################################################
    # Episode done?
    ####################################################################################################################
    if episode_done:
        # stop robot movement
        a_t[0][0] = 0.0
        a_t[0][1] = 0.0
        ros_handler.execute_action(a_t[0])

        end_time_episode = time()

        time_elapsed = end_time_episode - start_time_episode

        print("Start time: " + str(start_time_episode))
        print("End time: " + str(end_time_episode))
        print("Elapsed: " + str(time_elapsed))

        #build string for logging
        log_info = '' + str(ctime(start_time_episode)) + ',' + str(ctime(end_time_episode)) + ',' + str(time_elapsed) + \
                   ',' + str(step) + ',' + str(params.epsilon) + ',' + str(goal_pose.position.x) + \
                   ',' + str(goal_pose.position.y) + ',' + str(goal_pose.orientation.x) + \
                   ',' + str(goal_pose.orientation.y) + ',' + str(goal_pose.orientation.z) + \
                   ',' + str(goal_pose.orientation.w) + ',' + str(TimeOutOrAway) +\
                   ',' + str(gpl_generator.is_trajectory()) + ',' + str(params.gpl_index) + \
                   ',' + str(total_reward)
        #log all data from episode to file for evaluation
        f = open(str(params.log_path) + 'episode.log', 'a+')
        f.write(str(log_info) + "\n")
        f.close()

        TimeOutOrAway = False
        first_action_in_episode = True
        total_reward = 0.0

        #check if new weights are available after each X episodes
        if np.mod(episode, params.weight_update) == 0:
            action_generator.load_new_weights()

        return "ST_RESET_WORLD"

    ####################################################################################################################
    #First Action in Episode
    ####################################################################################################################
    if first_action_in_episode:
        print("Episode : " + str(episode))

        #check if state space transformations are available
        usable = ros_handler.is_state_space_usable()
        while not usable:
            usable = ros_handler.is_state_space_usable()

        # get starting state space
        [s_t, tmp] = ros_handler.get_state()

        action = action_generator.generate_action(np.asarray([s_t]))

        a_t_original_left = action[0][0]
        a_t_original_right = action[0][1]

        # generate exploration noise
        noise_t[0][0] = max(params.epsilon, 0) * noise_generator.get_exploration_noiseV2(a_t_original_left, 1)
        noise_t[0][1] = max(params.epsilon, 0) * noise_generator.get_exploration_noiseV2(a_t_original_right, 2)

        a_t[0][0] = (a_t_original_left + noise_t[0][0])
        a_t[0][1] = (a_t_original_right + noise_t[0][1])

        #limit actions to max output of ANN
        if a_t[0][0] > 1:
            a_t[0][0] = 1
        if a_t[0][1] > 1:
            a_t[0][1] = 1

        if a_t[0][0] < -1:
            a_t[0][0] = -1
        if a_t[0][1] < -1:
            a_t[0][1] = -1

        a_t_transition[0][0] = a_t[0][0]
        a_t_transition[0][1] = a_t[0][1]

        ros_handler.execute_action(a_t[0])

        print("Episode", episode, "Step", step, "Action", a_t, "Action_Original", a_t_original_left, " ",
              a_t_original_right, "Noise", noise_t, "Epsilon", params.epsilon)

        first_action_in_episode = False

        # calculate starting distance for reward calculation
        reward_calculator.calc_starting_distance_for_new_goal(robot_current_map_pose_stamped, goal_pose)

    ####################################################################################################################
    # NOT first Action in Episode
    ####################################################################################################################
    else:
        usable = ros_handler.is_state_space_usable()
        while not usable:
            usable = ros_handler.is_state_space_usable()
        [s_t1, robot_current_map_pose_stamped] = ros_handler.get_state()


        # calculate reward and check if episode is finished
        [r_t, episode_done, distance_to_goal] = reward_calculator.get_reward(robot_current_map_pose_stamped,
                                                                             goal_pose, s_t1)


        TimeOutOrAway = False
        if r_t > 900:
            print("TimeOut or too far away!")
            r_t = r_t - 999
            episode_done = False
            TimeOutOrAway = True

        r_t = r_t/10

        print("Reward: ", r_t)

        total_reward = total_reward + r_t
        print("total_reward: ", total_reward)


        # add data to replay buffer
        current_transition = Transition()
        current_transition.statespace_size = params.state_dim
        current_transition.actionspace_size = params.action_dim
        for data_s_t in s_t:
            current_transition.s_t.append(float(data_s_t))
        for data_a_t in a_t_transition:
            current_transition.a_t.append(float(data_a_t[0]))
            current_transition.a_t.append(float(data_a_t[1]))

        current_transition.reward = r_t

        for data_s_t1 in s_t1:
            current_transition.s_t1.append(float(data_s_t1))

        current_transition.done = episode_done
        pub_transition.publish(current_transition)


        states_tmp = np.zeros(params.state_dim)
        states_tmp[:] = s_t1[:]
        s_t[:] = states_tmp[:]


        if TimeOutOrAway == True:
            episode_done = True

        if not episode_done:

            #if current goal list is a trajectory
            if gpl_generator.is_trajectory():
                #print("Distance to goal: " + str(distance_to_goal))
                #check if current point is end of trajectory
                if not gpl_generator.is_point_end_of_trajectory():
                    # check if distance is < 1.5 [m]
                    if distance_to_goal < 1.5:
                        #print("Setting new goal")
                        #set new goal point
                        goal_pose = gpl_generator.get_goal_point(params.gpl_use_random_goal, params.gpl_use_random_orientation)
                        ros_handler.set_goal_pose(goal_pose)
                        #wait for new transformations
                        usable = ros_handler.is_state_space_usable()
                        #print("Waiting: " + str(time()))
                        while not usable:
                            usable = ros_handler.is_state_space_usable()

                        #get new s_t for action generation
                        [s_t, robot_current_map_pose_stamped] = ros_handler.get_state()

                        #calculate starting distance for reward calculation
                        reward_calculator.calc_starting_distance_for_new_goal(robot_current_map_pose_stamped, goal_pose)

            #get action from ANN
            action = action_generator.generate_action(np.asarray([s_t]))

            a_t_original_left = action[0][0]
            a_t_original_right = action[0][1]

            # generate exploration noise
            noise_t[0][0] = max(params.epsilon, 0) * noise_generator.get_exploration_noiseV2(a_t_original_left, 1)
            noise_t[0][1] = max(params.epsilon, 0) * noise_generator.get_exploration_noiseV2(a_t_original_right, 2)

            a_t[0][0] = (a_t_original_left + noise_t[0][0])
            a_t[0][1] = (a_t_original_right + noise_t[0][1])

            # limit actions to max output of ANN
            if a_t[0][0] > 1:
                a_t[0][0] = 1
            if a_t[0][1] > 1:
                a_t[0][1] = 1

            if a_t[0][0] < -1:
                a_t[0][0] = -1
            if a_t[0][1] < -1:
                a_t[0][1] = -1

            a_t_transition[0][0] = a_t[0][0]
            a_t_transition[0][1] = a_t[0][1]

            ros_handler.execute_action(a_t[0])

            print("Episode", episode, "Step", step, "Action", a_t, "Action_Original", a_t_original_left, " ",
                  a_t_original_right, "Noise", noise_t, "Epsilon", params.epsilon)

            # wait until next spin for action to take effect
        else:
            # episode done
            # stop robot movement
            a_t[0][0] = 0.0
            a_t[0][1] = 0.0
            ros_handler.execute_action(a_t[0])


    #print("Episode", episode, "Step", step)

    step = step + 1

    return "ST_EXECUTING"

def ST_RESET_WORLD():
    #print("ST_RESET_WORLD")

    ##########################################
    #              ros_handler               #
    ##########################################
    global ros_handler

    ##########################################
    #            obstacle_handler            #
    ##########################################
    global obstacle_handler

    ####Timing####
    global reset_world_cnt

    if reset_world_cnt == 0:
        print("Reset World")
        ros_handler.reset_world()
    if reset_world_cnt > 10:
        reset_world_cnt = 0
        return "ST_STARTUP"

    reset_world_cnt = reset_world_cnt + 1
    return "ST_RESET_WORLD"


#----------------------------------------------------------------------------->
#------------------------errorhandler----------------------------------------->
#----------------------------------------------------------------------------->
# Error handling
def errorhandler():
    #close this program
    print("--------------------------Critical Error-------------------------")
    #exit script
    sys.exit()



###############################################################################
#############################  MAIN  ##########################################
###############################################################################
if __name__ == '__main__':


    ##########################################
    #          parameter_handler             #
    ##########################################
    global params
    params = PH.param_handler()
    params.read_parameter(CONFIG_FILE)
    params.print_parameter()

    ##########################################
    #                ROS-Node                #
    ##########################################
    rospy.init_node('SimulationHandlerPython', anonymous=False)

    rate = rospy.Rate(params.control_rate)  #Hz

    #Topics
    global pub_transition
    pub_transition = rospy.Publisher('/multi_learning/transitions', Transition, queue_size=10)


    ##########################################
    #              ros_handler               #
    ##########################################
    global ros_handler
    ros_handler = REH.ros_environment(state_space_version=params.state_space_version, simulation=params.simulation,
                                      state_dim=params.state_dim ,wheel_diameter=params.wheel_diameter,
                                      axis_length=params.axis_length, additional_weight=0.0)

    ##########################################
    #            noise_generator             #
    ##########################################
    global noise_generator
    noise_generator = noise_gen.exploration_noise_generator(static_params=False,OU_mu=params.OU_mu,
                                                            OU_theta=params.OU_theta,
                                                            OU_sigma=params.OU_sigma)

    ##########################################
    #            action_generator            #
    ##########################################
    global action_generator
    action_generator = ANN.ANN(params.path_to_weights, params.weights_ID)

    if not action_generator.is_init():
        exit(999)


    ##########################################
    #           reward_calculator            #
    ##########################################
    global reward_calculator
    reward_calculator = RC.reward_calculator(params.goal_max_dist, params.episode_timeout,
                                             params.distance_reward_factor, params.use_crash_prevention, params.goal_area_x,
                                             params.goal_area_y, params.goal_area_angle)

    ##########################################
    #             gpl_generator              #
    ##########################################
    global gpl_generator
    gpl_generator = gpl_gen.goal_point_generator(params.gpl_index)


    ####StartUp####
    global startup_cnt
    startup_cnt = 0

    ####Episode Handling####
    global episode
    episode = 0

    global step
    step = 0

    global first_action_in_episode
    first_action_in_episode = True

    global episode_done
    episode_done = False

    global total_reward
    total_reward = 0.0

    ####Reset####
    global reset_world_cnt
    reset_world_cnt = 0


    ####Data Handling####
    global s_t
    s_t = np.zeros([params.state_dim])

    global a_t_transition
    a_t_transition = np.zeros([1, params.action_dim])

    global goal_pose
    goal_pose = pose_msg_type()
    # ros_handler.set_goal_pose(goal_pose)

    global robot_current_map_pose_stamped
    robot_current_map_pose_stamped = pose_stamped_msg_type()



    ##########################################
    #                 States                 #
    ##########################################
    stateAction = {
        "ST_STARTUP": ST_STARTUP,
        "ST_RESET_WORLD": ST_RESET_WORLD,
        "ST_EXECUTING": ST_EXECUTING,
    }

    # initial state
    activeState = "ST_STARTUP"

    #cycle time for states
    duration_max = np.float(np.float(1) / np.float(params.control_rate))

    try:
        while not rospy.is_shutdown():

            start_time = rospy.Time.now()
            oldState = activeState
            activeState = stateAction.get(activeState, errorhandler)()
            rate.sleep()
            end_time = rospy.Time.now()
            duration = end_time - start_time
            #print("Duration max: " + str(duration_max))
            if duration.to_sec() > duration_max:
                msg = 'Loop missed its desired rate of ' + str(params.control_rate) + 'Hz ... it took ' + str(duration.to_sec()) + ' seconds instead of max ' + str(duration_max) + ' seconds'
                rospy.logwarn(msg)
                #rospy.logwarn("Duration: " + str(duration.to_sec()))
    finally:
        # close connection, remove subcsriptions, etc
        print("ERROR CATCHED! -> Exit")
        print("Episodes handled: " + str(episode))
        print("Mean Episodes length: " + str("NOT IMPLEMENTED YET"))
        print("Mean total reward: " + str("NOT IMPLEMENTED YET"))
