# eCub_lenghtAndDensityWithHumanOptimization.py
# this file computes the set of optimum hardware parameters and robot, human and box configuration
# for ergonomic payload lifting task at different heights 


from Optimizer import Optimizer
from OptimizerHuman import OptimizerHuman
from OutputVisualization import OutputVisualization
from OptimizerHumanRobot import OptimizerHumanRobot
import ergoCubConfiguration 
import numpy as np
import time 
import math
import copy 
import os
import casadi as cs 

# hardware parameter part list 
hardwareOptimizationPartList = ['LeftArm','RightArm','LeftLeg','RightLeg', 'Torso']

# models name 
modifiedModel = "eCub"
originalModel = "iCub"
humanModel = "human"

# timeout visualization in seconds 
time_out_viz = 5 

# get common path
common_path = os.path.dirname(os.path.abspath(__file__)) + "/../"

# defining urdf paths
urdf_path =common_path + "/models/model.urdf"
urdf_path_modified = common_path+"/model_modified.urdf"
urdf_human_path = common_path+ "/models/humanSubject04_48dof.urdf"
urdf_box_path = common_path+"/models/box_idyn.urdf"

# defining controlled joint list 
joints_ctrl_name_list = [
    'torso_pitch', 'torso_roll', 'torso_yaw', 'l_shoulder_pitch',
    'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow', 'l_wrist_prosup', 'r_shoulder_pitch',
    'r_shoulder_roll', 'r_shoulder_yaw', 'r_elbow','r_wrist_prosup', 'l_hip_pitch', 'l_hip_roll',
    'l_hip_yaw', 'l_knee', 'l_ankle_pitch', 'l_ankle_roll', 'r_hip_pitch',
    'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll'
]

joints_human_list = [    'jT1C7_roty', 
                         'jT9T8_roty', 
                         'jL1T12_roty',
                         'jL4L3_roty', 
                         'jL5S1_roty', 

                         'jLeftC7Shoulder_rotx',
                         'jLeftShoulder_rotx','jLeftShoulder_roty','jLeftShoulder_rotz',
                         'jLeftElbow_roty','jLeftElbow_rotz', 
                         'jLeftWrist_rotx','jLeftWrist_rotz',

                         'jRightC7Shoulder_rotx', 
                         'jRightShoulder_rotx','jRightShoulder_roty','jRightShoulder_rotz',
                         'jRightElbow_roty','jRightElbow_rotz',
                         'jRightWrist_rotx','jRightWrist_rotz',

                         'jLeftHip_rotx','jLeftHip_roty',
                         'jLeftKnee_roty', 
                         'jLeftAnkle_rotx','jLeftAnkle_roty','jLeftAnkle_rotz',

                         'jRightHip_rotx','jRightHip_roty', 
                         'jRightKnee_roty', 
                         'jRightAnkle_rotx','jRightAnkle_roty','jRightAnkle_rotz']

# defining frame name list             
root_link = 'root_link'
right_hand_frame_human = 'RightHand'
left_hand_frame_human = 'LeftHand'
left_foot_frame_human = 'LeftToe'
right_foot_frame_human = 'RightToe'
contact_frames_human = [left_foot_frame_human, right_foot_frame_human, left_hand_frame_human, right_hand_frame_human]

right_hand_frame_robot = 'r_hand_frame'
left_hand_frame_robot = 'l_hand_frame'
left_foot_frame_robot = 'l_sole'
right_foot_frame_robot = 'r_sole'
contact_frames_robot = [left_foot_frame_robot, right_foot_frame_robot, left_hand_frame_robot, right_hand_frame_robot]

# defining box height considered
number_of_points =4
height_hands_start = [0.8, 1.0, 1.2,1.5]

# defining initial conditions 
length_initial =[1.00992, 1.44575, 1.41355, 1.14021, 0.513441, 1.34682, 0.874503, 1.10944]
quat_b_initial = [1.0,0.0, 0.0, 0.0,-0.0489,-0.0648,0.65]
s_initial = np.zeros(len(joints_ctrl_name_list))
initial_arm = [-0.7749,0.1920,0.0,0.7854, -1.5359]
s_initial[3:8] = initial_arm
s_initial[8:13] = initial_arm
s_initial_human = ergoCubConfiguration.get_initial_human_configuration(joints_human_list)
base_position_init = np.array([0, 0, 1.0])
base_orientationQuat_init = np.array([0, 0, 0, 1])
base_orientationQuat_position_init = np.concatenate((base_orientationQuat_init, base_position_init), axis=0)

s_initial_tot = []
quat_b_initial_tot = []
s_initial_human_tot= []
quat_b_inital_human_tot = []

for i in range(number_of_points): 
    if i == 1:
        s_initial_hoc = copy.deepcopy(s_initial)
        s_initial_hoc[0] = 30*math.pi/180
        s_initial_hoc[16] = -35*math.pi/180
        s_initial_hoc[22] = -35*math.pi/180
        s_initial_hoc[13] = 30*math.pi/180
        s_initial_hoc[19] = 30*math.pi/180
        s_initial_tot = cs.horzcat(s_initial_tot, s_initial_hoc)
    else:
       s_initial_tot = cs.horzcat(s_initial_tot, s_initial )
    quat_b_initial_tot = cs.horzcat(quat_b_initial_tot, quat_b_initial)

    s_initial_human_tot = cs.horzcat(s_initial_human_tot, s_initial_human)
    quat_b_inital_human_tot = cs.horzcat(base_orientationQuat_position_init, quat_b_inital_human_tot)


# defining set of feasible densities
target_densities = [2.81 *1000/2,  8*1000/2]

# defining the solver 
solver = cs.Opti()
p_opts = {}
s_opts = {'linear_solver':'ma27','hessian_approximation':'limited-memory'}
solver.solver("ipopt", p_opts,s_opts)

# setting if consider hardware parameters 
add_denstiy = True
consider_hardware_parameters = False
key_word = "optimized"

if consider_hardware_parameters: 
    add_denstiy = False
    key_word = "original"

# defining the optimizers 
OptimizerHelper = Optimizer(hardwareOptimizationPartList, urdf_path, joints_ctrl_name_list, root_link,contact_frames_robot, number_of_points, add_denstiy)
OptimizerHelperHuman = OptimizerHuman(urdf_human_path, joints_human_list,"Pelvis",contact_frames_human, number_of_points, solver)
OptimizerHelperHumanRobot =OptimizerHumanRobot(OptimizerHelper, OptimizerHelperHuman, solver, urdf_box_path)

number_of_figure = 0 


OptimizerHelper.set_solver(OptimizerHelperHumanRobot.solver)
density = np.abs(np.random.rand(OptimizerHelper.numberOfParameter))
OptimizerHelperHumanRobot.optimizerRobot.set_initial_with_variable(s_initial_tot, quat_b_initial_tot, length_initial, density)

# defining optimization problem 
OptimizerHelperHumanRobot.add_robot_to_optimizer(height_hands_start, target_densities, s_initial_tot, quat_b_initial_tot, compute_original=consider_hardware_parameters)
if(add_denstiy):
    OptimizerHelper.add_density_constraint(target_densities)
OptimizerHelperHumanRobot.add_human_to_optimizer(s_initial_human_tot, quat_b_inital_human_tot, height_hands_start)
OptimizerHelperHumanRobot.minimum_torque_cost()
OptimizerHelperHumanRobot.add_contact()
OptimizerHelperHumanRobot.add_box_constraint(height_hands_start)

# solving optimization problem
OptimizerHelperHumanRobot.solve_optimization_problem(urdf_path_modified)

# taking the output 
tau, wrenches = OptimizerHelperHumanRobot.compute_output_torque()        
wrenches_robot_feet = wrenches[:12,:]
wrenches_human_feet = wrenches[12:24,:]
wrenches_robot_hand = wrenches[24:30,:]
wrenches_human_hand = wrenches[30:,:]

# Visualizing the output 
VisualizationHelper = OutputVisualization(OptimizerHelper)
VisualizationHelper.prepare_visualization()


# if different box heights are considered 
if(OptimizerHelper.number_of_points>1):
   
    # adding the robot, human and box model 
    [index_left_foot, index_right_foot, index_com, index_right_hand, index_left_hand]=VisualizationHelper.add_and_update_model(modifiedModel, urdf_path_modified, joints_ctrl_name_list, root_link, OptimizerHelper.quat_b_opt[:,0], OptimizerHelper.s_opt_all[:,0],OptimizerHelper.density_optimum_vector, OptimizerHelper.length_optimum_vector)
    [index_left_foot_human, index_right_foot_human, index_com_human]=VisualizationHelper.add_and_update_model_no_hardware(humanModel, urdf_human_path, joints_human_list, "Pelvis", OptimizerHelperHuman.quat_b_opt[:,0], OptimizerHelperHuman.s_opt_all[:,0],OptimizerHelperHuman) 
    VisualizationHelper.add_box(urdf_box_path, "Box","base_link", OptimizerHelperHumanRobot.box_pose_opt[:,0])
    
    # adding the wrenches 
    idx_left_foot_human = VisualizationHelper.visualize_wrench(OptimizerHelperHuman.kinDyn,left_foot_frame_human, OptimizerHelperHuman.H_b_fun(OptimizerHelperHuman.quat_b_opt[:,0]), OptimizerHelperHuman.s_opt_all[:,0], 
    wrenches_human_feet[:6,0])
    idx_right_foot_human = VisualizationHelper.visualize_wrench(OptimizerHelperHuman.kinDyn,right_foot_frame_human, OptimizerHelperHuman.H_b_fun(OptimizerHelperHuman.quat_b_opt[:,0]), OptimizerHelperHuman.s_opt_all[:,0], 
    wrenches_human_feet[6:,0])
    idx_left_foot_robot = VisualizationHelper.visualize_wrench_param_model(OptimizerHelper.kinDyn,left_foot_frame_robot, OptimizerHelper.H_b_fun(OptimizerHelper.quat_b_opt[:,0]), OptimizerHelper.s_opt_all[:,0], 
    OptimizerHelper.length_optimum_vector,OptimizerHelper.density_optimum_vector, wrenches_robot_feet[:6,0])
    idx_right_foot_robot = VisualizationHelper.visualize_wrench_param_model(OptimizerHelper.kinDyn,right_foot_frame_robot, OptimizerHelper.H_b_fun(OptimizerHelper.quat_b_opt[:,0]), OptimizerHelper.s_opt_all[:,0], 
    OptimizerHelper.length_optimum_vector,OptimizerHelper.density_optimum_vector, wrenches_robot_feet[6:,0])
    
    # visualizing the different robot, human and box configurations
    for item in range(OptimizerHelper.number_of_points):
        
        # updating the models 
        VisualizationHelper.update_model("eCub", OptimizerHelper.quat_b_opt[:,item], OptimizerHelper.s_opt_all[:,item], index_left_foot, index_right_foot, index_com, index_left_hand, index_right_hand,OptimizerHelper.density_optimum_vector, OptimizerHelper.length_optimum_vector)
        VisualizationHelper.update_model_no_hardware(OptimizerHelperHuman,humanModel, OptimizerHelperHuman.quat_b_opt[:,item], OptimizerHelperHuman.s_opt_all[:,item], index_left_foot_human, index_right_foot_human, index_com_human)
        VisualizationHelper.add_box(urdf_box_path, "Box","base_link", OptimizerHelperHumanRobot.box_pose_opt[:,item], False)
        
        # visualizing the wrenches 
        VisualizationHelper.visualize_wrench(OptimizerHelperHuman.kinDyn,left_foot_frame_human, OptimizerHelperHuman.H_b_fun(OptimizerHelperHuman.quat_b_opt[:,item]), OptimizerHelperHuman.s_opt_all[:,item], 
        wrenches_human_feet[:6,item], idx_left_foot_human)
        VisualizationHelper.visualize_wrench(OptimizerHelperHuman.kinDyn,right_foot_frame_human, OptimizerHelperHuman.H_b_fun(OptimizerHelperHuman.quat_b_opt[:,item]), OptimizerHelperHuman.s_opt_all[:,item], 
        wrenches_human_feet[6:,item], idx_right_foot_human)
        VisualizationHelper.visualize_wrench_param_model(OptimizerHelper.kinDyn,left_foot_frame_robot, OptimizerHelper.H_b_fun(OptimizerHelper.quat_b_opt[:,item]), OptimizerHelper.s_opt_all[:,item], 
        OptimizerHelper.length_optimum_vector,OptimizerHelper.density_optimum_vector, wrenches_robot_feet[:6,item], idx_left_foot_robot)
        VisualizationHelper.visualize_wrench_param_model(OptimizerHelper.kinDyn,right_foot_frame_robot, OptimizerHelper.H_b_fun(OptimizerHelper.quat_b_opt[:,item]), OptimizerHelper.s_opt_all[:,item], 
        OptimizerHelper.length_optimum_vector,OptimizerHelper.density_optimum_vector, wrenches_robot_feet[6:,item], idx_right_foot_robot)
        
        # output the visualization 
        time_now = time.time()
        while((time.time()-time_now)<time_out_viz and VisualizationHelper.viz.run()):
            VisualizationHelper.viz.draw()

        # saving the figures 
        for jj in range(10):
            number_of_figure +=1
            file_name = common_path+"/robotOutput/Output_"+key_word+str(number_of_figure)+".png"
            VisualizationHelper.viz.drawToFile(file_name)

# if only one box height is considered
else: 

    # adding and update the robot, human and box models 
    [index_left_foot, index_right_foot, index_com, index_right_hand, index_left_hand]=VisualizationHelper.add_and_update_model(modifiedModel, urdf_path_modified, joints_ctrl_name_list, root_link, OptimizerHelper.quat_b_opt, OptimizerHelper.s_opt_all,OptimizerHelper.density_optimum_vector, OptimizerHelper.length_optimum_vector)
    [index_left_foot, index_right_foot, index_com]=VisualizationHelper.add_and_update_model_no_hardware(humanModel, urdf_human_path, joints_human_list, "Pelvis", OptimizerHelperHuman.quat_b_opt, OptimizerHelperHuman.s_opt_all,OptimizerHelperHuman) 
    VisualizationHelper.add_box(urdf_box_path, "Box","base_link", OptimizerHelperHumanRobot.box_pose_opt)
    
    # output the visualization 
    time_now = time.time()
    while(time.time()-time_now<time_out_viz and VisualizationHelper.viz.run()): 
        VisualizationHelper.viz.draw()
    
    # saving the figures 
    for jj in range(10):
            number_of_figure +=1
            file_name = common_path+"/robotOutput/Output_"+str()+".png"
            VisualizationHelper.viz.drawToFile(file_name)
