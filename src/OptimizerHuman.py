from adam.casadi.computations import KinDynComputations
import utils as eCubUtils
import casadi as cs
import numpy as np


class OptimizerHuman:
    def __init__(
        self, model, joint_ctr_list, root_link, contact_frames, number_points, solver
    ) -> None:
        self.model = model
        self.joint_ctr_list = joint_ctr_list
        self.number_of_points = number_points
        self.root_link = root_link
        self.solver = solver
        self.N_DOF = len(self.joint_ctr_list)
        self.kinDyn = KinDynComputations(
            self.model, self.joint_ctr_list, self.root_link
        )
        self.cost_function = 0.0
        self.contact_frames = contact_frames
        self.define_kin_din_functions()
        self.initialize_search_variable()

    def define_kin_din_functions(self):
        print("***** Defining kin and din functions*****")
        # Frames of Interest
        right_hand_frame = "RightHand"
        left_hand_frame = "LeftHand"
        left_foot_frame = "LeftToe"
        right_foot_frame = "RightToe"
        frame_heigth = "head"

        self.M = self.kinDyn.mass_matrix_fun()
        self.J_left = self.kinDyn.jacobian_fun(left_foot_frame)
        self.J_right = self.kinDyn.jacobian_fun(right_foot_frame)
        self.J_right_hand = self.kinDyn.jacobian_fun(right_hand_frame)
        self.J_left_hand = self.kinDyn.jacobian_fun(left_hand_frame)
        self.g = self.kinDyn.gravity_term_fun()
        self.B = cs.vertcat(cs.MX.zeros(6, self.N_DOF), cs.MX.eye(self.N_DOF))
        self.error_rot, self.error_pos = eCubUtils.TransfMatrixError()
        self.H_right_hand = self.kinDyn.forward_kinematics_fun(right_hand_frame)
        self.H_left_hand = self.kinDyn.forward_kinematics_fun(left_hand_frame)
        self.H_left_foot = self.kinDyn.forward_kinematics_fun(left_foot_frame)
        self.H_right_foot = self.kinDyn.forward_kinematics_fun(right_foot_frame)
        self.w_H_torso = self.kinDyn.forward_kinematics_fun(self.root_link)
        self.H_b_fun = eCubUtils.FromQuaternionToMatrix()
        self.theta_left_foot_error = eCubUtils.zAxisAngle()
        self.CoM = self.kinDyn.CoM_position_fun()
        self.total_mass = self.kinDyn.get_total_mass()

    def compute_contact_jacobian(self, H_b_i, s_i):
        Jc = []
        for item in self.contact_frames:
            J = self.kinDyn.jacobian_fun(item)
            Jc = cs.vertcat(Jc, J(H_b_i, s_i))
        return Jc

    def initialize_search_variable(self):

        # Define the variable of the optimization problem
        self.s = self.solver.variable(
            self.N_DOF, self.number_of_points
        )  # joint positions
        self.quat_pose_b = self.solver.variable(
            7, self.number_of_points
        )  # base pose as quaternion

        # Define the parameters of the optimization problem
        self.H_right_hand_star = self.solver.parameter(4, 4)
        self.H_left_hand_star = self.solver.parameter(4, 4)
        self.H_left_foot_star = self.solver.parameter(4, 4)
        self.H_right_foot_star = self.solver.parameter(4, 4)
        self.q_zero = self.solver.parameter(self.N_DOF)

    def set_target_height_hands(self, height_hand_star):
        self.height_hands_start = height_hand_star

    def add_torque_minimization(self, H_b_i, s_i):
        # Defining torque minimization
        Jc = cs.vertcat(self.J_left(H_b_i, s_i), self.J_right(H_b_i, s_i))
        Jc = self.compute_contact_jacobian(H_b_i, s_i)
        M_param = self.M(H_b_i, s_i)
        J_weigthed_mass = Jc @ cs.inv(M_param) @ cs.transpose(Jc)
        N_A_first_part = cs.transpose(Jc) @ cs.inv(J_weigthed_mass)
        N_A_second_part = Jc @ cs.inv(M_param)
        N_A = cs.MX.eye(self.N_DOF + 6) - N_A_first_part @ N_A_second_part
        tau = cs.pinv(N_A @ self.B) @ N_A @ self.g(H_b_i, s_i)
        weight_tau = 1.0
        # Tau
        cost_function_tau = weight_tau * cs.sumsqr(tau)
        self.cost_function += cost_function_tau

    def add_constraint_cost_hands(self, H_b_i, s_i, height_star):

        weigth_right_hand = [0.0, 1.0]  # [rot,pos]
        weigth_left_hand = [0.0, 1.0]  # [rot,pos]

        # Target hands height
        # Right
        H_right_hands_param = self.H_right_hand(H_b_i, s_i)
        error_right_hand_heights = cs.sumsqr(H_right_hands_param[2, 3] - height_star)

        # Target hands height
        # Left
        H_left_hands_param = self.H_left_hand(H_b_i, s_i)
        error_left_hand_heights = cs.sumsqr(H_left_hands_param[2, 3] - height_star)

        # Target hands orientation
        # Right
        error_rot_right_hand = self.error_rot(
            self.H_right_hand(H_b_i, s_i), self.H_right_hand_star
        )
        cost_function_rotation_right_hand = weigth_right_hand[1] * cs.sumsqr(
            error_rot_right_hand
        )

        # Left
        error_rot_left_hand = self.error_rot(
            self.H_left_hand(H_b_i, s_i), self.H_left_hand_star
        )
        cost_function_rotation_left_hand = weigth_left_hand[1] * cs.sumsqr(
            error_rot_left_hand
        )

    def add_feet_constraint(self, H_b_i, s_i):
        H_param_left_Foot = self.H_left_foot(H_b_i, s_i)
        H_param_right_foot = self.H_right_foot(H_b_i, s_i)
        self.solver.subject_to(self.theta_left_foot_error(H_param_right_foot) == 0.0)
        self.solver.subject_to(self.theta_left_foot_error(H_param_left_Foot) == 0.0)
        self.solver.subject_to(H_param_right_foot[2, 3] == self.H_right_foot_star[2, 3])
        self.solver.subject_to(H_param_left_Foot[2, 3] == self.H_left_foot_star[2, 3])

    def add_intrinsic_constraint(self, q_base_i, s_i):
        self.solver.subject_to(cs.sumsqr(q_base_i[:4]) == 1.0)

    def set_solutions(self, sol):
        self.s_opt_all = sol.value(self.s)
        self.quat_b_opt = sol.value(self.quat_pose_b)

    def set_references(self):

        ## Desired quantities
        w_H_torso_num = self.w_H_torso(np.eye(4), self.s_initial[:, 0])
        w_H_lefFoot_num = self.H_left_foot(np.eye(4), self.s_initial[:, 0])
        w_H_init = cs.inv(w_H_lefFoot_num) @ w_H_torso_num
        self.w_H_lFoot_des = self.H_left_foot(w_H_init, self.s_initial[:, 0])
        self.w_H_rFoot_des = self.H_right_foot(w_H_init, self.s_initial[:, 0])
        self.w_H_lHand_des = self.H_left_hand(w_H_init, self.s_initial[:, 0])
        self.w_H_rHand_des = self.H_right_hand(w_H_init, self.s_initial[:, 0])
        # Setting the values
        self.solver.set_value(self.q_zero, np.zeros(self.N_DOF))

        self.solver.set_value(self.H_left_foot_star, self.w_H_lFoot_des)
        self.solver.set_value(self.H_right_foot_star, self.w_H_rFoot_des)
        self.solver.set_value(self.H_left_hand_star, self.w_H_lHand_des)
        self.solver.set_value(self.H_right_hand_star, self.w_H_rHand_des)

    def set_initial_with_variable(self, s_initial, quat_b_initial):

        self.s_initial = s_initial
        self.quat_b_initial = quat_b_initial
        self.solver.set_initial(self.s, self.s_initial)
        self.solver.set_initial(self.quat_pose_b, self.quat_b_initial)

    def populating_optimization_problem(self):
        for i in range(self.number_of_points):
            s_i = self.s[:, i]
            q_base_i = self.quat_pose_b[:, i]
            H_b_i = self.H_b_fun(q_base_i)
            height_star = self.height_hands_start[i]
            self.add_constraint_cost_hands(H_b_i, s_i, height_star)
            self.add_feet_constraint(H_b_i, s_i)
            self.add_intrinsic_constraint(q_base_i, s_i)

    def add_joints_limits(self, s_i):
        joints_limits = {
            "jL5S1_rotx": [-0.610865, 0.610865],
            "jL5S1_roty": [-0.523599, 0.3], 
            "jL4L3_rotx": [-0.610865, 0.610865],
            "jL4L3_roty": [-0.523599, 0.3], 
            "jL1T12_rotx": [-0.610865, 0.610865],
            "jL1T12_roty": [-0.523599, 0.3],
            "jT9T8_rotx": [-0.349066, 0.349066],
            "jT9T8_roty": [-0.261799, 0.3], 
            "jT9T8_rotz": [-0.610865, 0.610865],
            "jT1C7_rotx": [-0.610865, 0.610865],
            "jT1C7_roty": [-0.959931, 0.3], 
            "jT1C7_rotz": [-1.22173, 1.22173],
            "jC1Head_rotx": [-0.610865, 0.610865],
            "jC1Head_roty": [-0.436332, 0.174533],
            "jRightC7Shoulder_rotx": [-0.785398, 0.0872665],
            "jRightShoulder_rotx": [-2.35619, 1.5708],
            "jRightShoulder_roty": [-1.5708, 1.5708],
            "jRightShoulder_rotz": [-0.785398, 3.14159],
            "jRightElbow_roty": [-1.5708, 1.48353],
            "jRightElbow_rotz": [0, 2.53073],
            "jRightWrist_rotx": [-0.872665, 1.0472],
            "jRightWrist_rotz": [-0.523599, 0.349066],
            "jLeftC7Shoulder_rotx": [-0.0872665, 0.785398],
            "jLeftShoulder_rotx": [-1.5708, 2.35619],
            "jLeftShoulder_roty": [-1.5708, 1.5708],
            "jLeftShoulder_rotz": [-3.14159, 0.785398],
            "jLeftElbow_roty": [-1.5708, 1.48353],
            "jLeftElbow_rotz": [-2.53073, 0.0],
            "jLeftWrist_rotx": [-1.0472, 0.872665],
            "jLeftWrist_rotz": [-0.349066, 0.523599],
            "jRightHip_rotx": [-0.785398, 0.523599],
            "jRightHip_roty": [-2.0, -0.261799],
            "jRightHip_rotz": [-0.785398, 0.785398],
            "jRightKnee_roty": [0.01, 2.35619],
            "jRightKnee_rotz": [-0.698132, 0.523599], 
            "jRightAnkle_rotx": [0, 0.785398],
            "jRightAnkle_roty": [-0.523599, 0.872665],
            "jRightAnkle_rotz": [-0.436332, 0.872665],
            "jLeftHip_rotx": [-0.523599, 0.785398],
            "jLeftHip_roty": [-1.0, -0.261799],
            "jLeftHip_rotz": [-0.785398, 0.785398],
            "jLeftKnee_roty": [0.01, 2.35619],
            "jLeftKnee_rotz": [-0.523599, 0.698132],
            "jLeftAnkle_rotx": [-0.785398, 0], 
            "jLeftAnkle_roty": [-0.523599, 0.872665],
            "jLeftAnkle_rotz": [-0.872665, 0.436332],
        }
        for i in range(self.N_DOF):
            item = self.joint_ctr_list[i]
            joints_limits_i = joints_limits[item]
            self.solver.subject_to(s_i[i] < joints_limits_i[1])
            self.solver.subject_to(s_i[i] > joints_limits_i[0])
