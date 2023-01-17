import ergoCubConfiguration
from adam.core import link_parametric
from adam.casadi.computations import KinDynComputations
import utils as eCubUtils
import casadi as cs
import numpy as np
import math
import urdfModifiers.utils as urdfModifiers
import utils as eCubUtils


class Optimizer:
    def __init__(
        self,
        partsList,
        urdf_path,
        joints_ctrl_name_list,
        root_link,
        contact_frame,
        number_of_points=3,
        densityParametrization=False,
    ) -> None:
        self.partsList = partsList
        self.LinkCharacteristics = {}
        self.JointCharacteristics = {}
        self.joint_name_list = []
        self.link_name_list = []
        self.index_parts = {}
        self.urdf_path = urdf_path
        self.joints_ctrl_name = joints_ctrl_name_list
        self.root_link = root_link
        self.N_DoF = len(joints_ctrl_name_list)
        self.number_of_points = number_of_points
        self.densityParametrization = densityParametrization
        self.contact_frame = contact_frame

        for item in self.partsList:
            (
                link_name_list_temp,
                joint_name_list_temp,
                links_characteristics_temp,
                joint_characteristics_temp,
            ) = ergoCubConfiguration.get_link_join_char(item)
            self.LinkCharacteristics.update(links_characteristics_temp)
            self.JointCharacteristics.update(joint_characteristics_temp)
            original_len = len(self.link_name_list)
            self.link_name_list += link_name_list_temp
            self.joint_name_list += joint_characteristics_temp
            index_parts = {item: [original_len, len(link_name_list_temp)]}

        self.numberOfParameter = (
            ergoCubConfiguration.NumberHardwareParameterForSymmetry(partsList)
        )
        self.kinDyn = KinDynComputations(
            self.urdf_path,
            self.joints_ctrl_name,
            self.root_link,
            self.link_name_list,
            self.LinkCharacteristics,
            self.JointCharacteristics,
        )
        print("******** Creating the ADAM kin din object********")
        print("- Robot Model = ", self.urdf_path)
        print("- Number of Points = ", self.number_of_points)
        print("- Links Considered = ", self.link_name_list)
        self.height_hands_start = [0.8, 1.0, 1.2]
        self.solver_initialize = False
        self.cost_function = 0.0
        self.define_kin_din_functions()
        self.compute_minimum_set_hardware_parameters()
        self.fc = []

    def set_solver(self, solver):
        self.solver = solver
        self.solver_initialize = True
        self.initialize_parameter_search_variable()

    def initialize_parameter_search_variable(self):
        # Define the variable of the optimization problem
        self.lengths_multipliers = self.solver.variable(self.numberOfParameter)
        # self.solver.set_value(self.lengths_multipliers, np.ones(self.numberOfParameter))
        self.s = self.solver.variable(
            self.N_DoF, self.number_of_points
        )  # joint positions
        self.quat_pose_b = self.solver.variable(
            7, self.number_of_points
        )  # base pose as quaternion
        self.lengths_multipliers_vector = (
            ergoCubConfiguration.CreateTotalHardwareParameterVector(
                self.partsList, self.lengths_multipliers
            )
        )

        # Define the parameters of the optimization problem
        if self.densityParametrization:
            self.density_var = self.solver.variable(self.numberOfParameter)
        else:
            self.density_var = self.solver.parameter(self.numberOfParameter)

        self.density = ergoCubConfiguration.CreateTotalHardwareParameterVector(
            self.partsList, self.density_var
        )
        self.H_right_hand_star = self.solver.parameter(4, 4)
        self.H_left_hand_star = self.solver.parameter(4, 4)
        self.H_left_foot_star = self.solver.parameter(4, 4)
        self.H_right_foot_star = self.solver.parameter(4, 4)
        self.q_zero = self.solver.parameter(self.N_DoF)
        self.lengths_multipliers_vector = (
            ergoCubConfiguration.CreateTotalHardwareParameterVector(
                self.partsList, self.lengths_multipliers
            )
        )

    def initialize_solver(self):
        self.solver = cs.Opti()
        # Define the solver
        p_opts = {}
        s_opts = {"linear_solver": "ma27"}
        self.solver.solver("ipopt", p_opts, s_opts)
        self.solver_initialize = True
        self.initialize_parameter_search_variable()

    def set_target_height_hands(self, height_hand_star):
        self.height_hands_start = height_hand_star

    def define_kin_din_functions(self):

        print("***** Defining kin and dyn functions*****")
        # Frames of Interest
        right_hand_frame = "r_hand_frame"
        left_hand_frame = "l_hand_frame"
        left_foot_frame = "l_sole"
        right_foot_frame = "r_sole"
        frame_heigth = "head"

        self.M = self.kinDyn.mass_matrix_fun()
        self.J_left = self.kinDyn.jacobian_fun(left_foot_frame)
        self.J_right = self.kinDyn.jacobian_fun(right_foot_frame)
        self.J_right_hand = self.kinDyn.jacobian_fun(right_hand_frame)
        self.J_left_hand = self.kinDyn.jacobian_fun(left_hand_frame)
        self.g = self.kinDyn.gravity_term_fun()
        self.B = cs.vertcat(cs.MX.zeros(6, self.N_DoF), cs.MX.eye(self.N_DoF))
        self.error_rot, self.error_pos = eCubUtils.TransfMatrixError()
        self.H_right_hand = self.kinDyn.forward_kinematics_fun(right_hand_frame)
        self.H_left_hand = self.kinDyn.forward_kinematics_fun(left_hand_frame)
        self.H_left_foot = self.kinDyn.forward_kinematics_fun(left_foot_frame)
        self.H_right_foot = self.kinDyn.forward_kinematics_fun(right_foot_frame)
        self.w_H_torso = self.kinDyn.forward_kinematics_fun(self.root_link)
        self.H_height = self.kinDyn.forward_kinematics_fun(frame_heigth)

        self.theta_left_foot_error = eCubUtils.zAxisAngle()

        self.CoM = self.kinDyn.CoM_position_fun()
        self.total_mass = self.kinDyn.get_total_mass()

        self.H_b_fun = eCubUtils.FromQuaternionToMatrix()

    def compute_contact_jacobian(self, H_b_i, s_i):
        Jc = []
        for item in self.contact_frame:
            J = self.kinDyn.jacobian_fun(item)
            Jc = cs.vertcat(
                Jc, J(H_b_i, s_i, self.density, self.lengths_multipliers_vector)
            )
        return Jc

    def add_torque_minimization(self, H_b_i, s_i):
        Jc = self.compute_contact_jacobian(H_b_i, s_i)
        # defining torque minimization
        M_param = self.M(H_b_i, s_i, self.density, self.lengths_multipliers_vector)
        J_weigthed_mass = Jc @ cs.inv(M_param) @ cs.transpose(Jc)
        N_A_first_part = cs.transpose(Jc) @ cs.inv(J_weigthed_mass)
        N_A_second_part = Jc @ cs.inv(M_param)
        N_A = cs.MX.eye(self.N_DoF + 6) - N_A_first_part @ N_A_second_part
        tau = (
            cs.pinv(N_A @ self.B)
            @ N_A
            @ self.g(H_b_i, s_i, self.density, self.lengths_multipliers_vector)
        )
        weight_tau = 1.0
        # Tau
        cost_function_tau = weight_tau * cs.sumsqr(tau)
        cost_function_postural = cs.sumsqr(s_i)
        self.cost_function += cost_function_tau
        fc = J_weigthed_mass @ (
            Jc
            @ cs.inv(M_param)
            @ (
                -self.B @ tau
                + self.g(H_b_i, s_i, self.density, self.lengths_multipliers_vector)
            )
        )
        self.fc = cs.horzcat(self.fc, fc)

    def add_whrenches_constraint(self, f_i):
        torsional_friction = 1 / 75
        static_friction = 1 / 3
        min_wrench = 0.0
        self.solver.subject_to(f_i[5] - torsional_friction * f_i[2] < 0)
        self.solver.subject_to(-f_i[5] - torsional_friction * f_i[2] < 0)
        self.solver.subject_to(f_i[2] > min_wrench)
        self.solver.subject_to(f_i[0] < static_friction * f_i[2])
        self.solver.subject_to(-f_i[0] < static_friction * f_i[2])
        self.solver.subject_to(f_i[1] < static_friction * f_i[2])
        self.solver.subject_to(-f_i[1] < static_friction * f_i[2])

    def add_constraint_cost_hands(self, H_b_i, s_i, height_star):

        if not (self.solver_initialize):
            print(
                "Solver not initialized !! please either set the solver via set_solver or intialize the solver via initialize_solver"
            )
            return

        weigth_right_hand = [0.0, 1.0]  # [rot,pos]
        weigth_left_hand = [0.0, 1.0]  # [rot,pos]

        # Target hands height
        # Right
        H_right_hands_param = self.H_right_hand(
            H_b_i, s_i, self.density, self.lengths_multipliers_vector
        )
        error_right_hand_heights = cs.sumsqr(H_right_hands_param[2, 3] - height_star)

        # Target hands height
        # Left
        H_left_hands_param = self.H_left_hand(
            H_b_i, s_i, self.density, self.lengths_multipliers_vector
        )
        error_left_hand_heights = cs.sumsqr(H_left_hands_param[2, 3] - height_star)

        # Target hands orientation
        # Right
        error_rot_right_hand = self.error_rot(
            self.H_right_hand(
                H_b_i, s_i, self.density, self.lengths_multipliers_vector
            ),
            self.H_right_hand_star,
        )
        cost_function_rotation_right_hand = weigth_right_hand[1] * cs.sumsqr(
            error_rot_right_hand
        )

        # Left
        error_rot_left_hand = self.error_rot(
            self.H_left_hand(H_b_i, s_i, self.density, self.lengths_multipliers_vector),
            self.H_left_hand_star,
        )
        cost_function_rotation_left_hand = weigth_left_hand[1] * cs.sumsqr(
            error_rot_left_hand
        )

    def add_com_cost(self):
        weigth_com = 1.0
        w_H_b_left_foot = self.H_left_foot(
            np.eye(4), self.q_zero, self.density, self.lengths_multipliers_vector
        )
        w_H_torso_zero_num = self.w_H_torso(
            np.eye(4),
            np.zeros(self.N_DoF),
            self.density,
            self.lengths_multipliers_vector,
        )
        w_H_b_zero = cs.mtimes(cs.inv(w_H_b_left_foot), w_H_torso_zero_num)
        CoM_param = self.CoM(
            w_H_b_zero,
            np.zeros(self.N_DoF),
            self.density,
            self.lengths_multipliers_vector,
        )
        cost_function_com = weigth_com * (1 / CoM_param[2])
        self.cost_function += cost_function_com

    def add_feet_constraint(self, H_b_i, s_i):

        if not (self.solver_initialize):
            print(
                "Solver not initialized !! please either set the solver via set_solver or intialize the solver via initialize_solver"
            )
            return

        H_param_left_Foot = self.H_left_foot(
            H_b_i, s_i, self.density, self.lengths_multipliers_vector
        )
        H_param_right_foot = self.H_right_foot(
            H_b_i, s_i, self.density, self.lengths_multipliers_vector
        )
        self.solver.subject_to(self.theta_left_foot_error(H_param_right_foot) == 0.0)
        self.solver.subject_to(self.theta_left_foot_error(H_param_left_Foot) == 0.0)
        self.solver.subject_to(H_param_right_foot[2, 3] == self.H_right_foot_star[2, 3])
        self.solver.subject_to(H_param_left_Foot[2, 3] == self.H_left_foot_star[2, 3])

    def add_intrinsic_constraint(self, q_base_i, s_i):

        if not (self.solver_initialize):
            print(
                "Solver not initialized !! please either set the solver via set_solver or intialize the solver via initialize_solver"
            )
            return

        self.solver.subject_to(cs.sumsqr(q_base_i[:4]) == 1.0)

    def add_mass_cost(self):
        # Arms
        Arms = ["LeftArm", "RightArm"]
        ArmsLink_List = ergoCubConfiguration.GetLinkListPart(Arms)
        print(ArmsLink_List)
        mass_arm = eCubUtils.ComputeTotalMassLinkChain(
            ArmsLink_List, self.kinDyn, self.density, self.lengths_multipliers_vector
        )

        # Torso
        TorsoLink_List = ergoCubConfiguration.GetLinkListPart(["Torso"])
        mass_torso = eCubUtils.ComputeTotalMassLinkChain(
            TorsoLink_List, self.kinDyn, self.density, self.lengths_multipliers_vector
        )

        # Legs
        Legs = ["LeftLeg", "RightLeg"]
        LegsLink_List = ergoCubConfiguration.GetLinkListPart(Legs)
        mass_legs = eCubUtils.ComputeTotalMassLinkChain(
            LegsLink_List, self.kinDyn, self.density, self.lengths_multipliers_vector
        )

        # Hands
        Hands = ["LeftHand", "RightHand"]
        HandsLink_List = ergoCubConfiguration.GetLinkListPart(Hands)
        mass_hands = eCubUtils.ComputeTotalMassLinkChain(
            HandsLink_List, self.kinDyn, self.density, self.lengths_multipliers_vector
        )

        # Feet
        Feet = ["LeftFoot", "RightFoot"]
        FeetLink_List = ergoCubConfiguration.GetLinkListPart(Feet)
        mass_feet = eCubUtils.ComputeTotalMassLinkChain(
            FeetLink_List, self.kinDyn, self.density, self.lengths_multipliers_vector
        )

        # Head
        HeadLink_List = ergoCubConfiguration.GetLinkListPart(["Head"])
        mass_head = eCubUtils.ComputeTotalMassLinkChain(
            HeadLink_List, self.kinDyn, self.density, self.lengths_multipliers_vector
        )

        total_mass_param = self.total_mass(
            self.density, self.lengths_multipliers_vector
        )
        cost_function_mass = (
            cs.sumsqr(mass_arm - 0.08 * total_mass_param)
            + cs.sumsqr(mass_legs - 0.32 * total_mass_param)
            + cs.sumsqr(mass_torso - 0.51 * total_mass_param)
        )
        self.cost_function += cost_function_mass

    def compute_original_density(self):
        density_vector_min = []
        # Compute original density
        for item in self.minimum_set_parameters_links:
            density_vector_min += [eCubUtils.ComputeOriginalDensity(self.kinDyn, item)]
        self.density_vector = ergoCubConfiguration.CreateTotalHardwareParameterVector(
            self.partsList, density_vector_min
        )
        self.density_vector_min = density_vector_min

    def set_references(self):
        if not (self.densityParametrization):
            self.compute_original_density()
            self.solver.set_value(self.density_var, self.density_vector_min)

        # Desired quantities
        number_of_links = len(self.link_name_list)
        w_H_torso_num = self.w_H_torso(
            np.eye(4),
            self.s_initial[:, 0],
            self.density_vector,
            np.ones(number_of_links),
        )
        w_H_lefFoot_num = self.H_left_foot(
            np.eye(4),
            self.s_initial[:, 0],
            self.density_vector,
            np.ones(number_of_links),
        )
        w_H_init = cs.inv(w_H_lefFoot_num) @ w_H_torso_num
        self.w_H_lFoot_des = self.H_left_foot(
            w_H_init,
            self.s_initial[:, 0],
            self.density_vector,
            np.ones(number_of_links),
        )
        self.w_H_rFoot_des = self.H_right_foot(
            w_H_init,
            self.s_initial[:, 0],
            self.density_vector,
            np.ones(number_of_links),
        )
        self.w_H_lHand_des = self.H_left_hand(
            w_H_init,
            self.s_initial[:, 0],
            self.density_vector,
            np.ones(number_of_links),
        )
        self.w_H_rHand_des = self.H_right_hand(
            w_H_init,
            self.s_initial[:, 0],
            self.density_vector,
            np.ones(number_of_links),
        )
        # Setting the values
        self.solver.set_value(self.q_zero, np.zeros(self.N_DoF))

        self.solver.set_value(self.H_left_foot_star, self.w_H_lFoot_des)
        self.solver.set_value(self.H_right_foot_star, self.w_H_rFoot_des)
        self.solver.set_value(self.H_left_hand_star, self.w_H_lHand_des)
        self.solver.set_value(self.H_right_hand_star, self.w_H_rHand_des)

    def set_initial_with_variable(
        self, s_initial, quat_b_initial, lenght_multiplier_initial, density_initial=None
    ):

        if not (self.solver_initialize):
            print(
                "Solver not initialized !! please either set the solver via set_solver or intialize the solver via initialize_solver"
            )
            return

        self.s_initial = s_initial
        self.quat_b_initial = quat_b_initial
        self.lenght_multiplier_initial = lenght_multiplier_initial
        self.solver.set_initial(self.s, self.s_initial)
        self.solver.set_initial(self.quat_pose_b, self.quat_b_initial)
        self.solver.set_initial(self.lengths_multipliers, lenght_multiplier_initial)
        if self.densityParametrization:
            self.density_initial = density_initial
            self.solver.set_initial(self.density_var, self.density_initial)

    def compute_minimum_set_hardware_parameters(self):
        self.minimum_set_parameters_links = []
        if "RightArm" in self.partsList or "LeftArm" in self.partsList:
            [armsLinkOptimizedList, _, _, _] = ergoCubConfiguration.get_link_join_char(
                "RightArm"
            )
            self.minimum_set_parameters_links += armsLinkOptimizedList
        if "LeftLeg" in self.partsList or "RightLeg" in self.partsList:
            [legsLinkOptimizedList, _, _, _] = ergoCubConfiguration.get_link_join_char(
                "LeftLeg"
            )
            self.minimum_set_parameters_links += legsLinkOptimizedList
        if "Torso" in self.partsList:
            [torsoLinkOptimizedList, _, _, _] = ergoCubConfiguration.get_link_join_char(
                "Torso"
            )
            self.minimum_set_parameters_links += torsoLinkOptimizedList

    def populating_optimization_problem(self):
        for i in range(self.number_of_points):
            s_i = self.s[:, i]
            q_base_i = self.quat_pose_b[:, i]
            H_b_i = self.H_b_fun(q_base_i)
            height_star = self.height_hands_start[i]
            self.add_constraint_cost_hands(H_b_i, s_i, height_star)
            self.add_feet_constraint(H_b_i, s_i)
            self.add_intrinsic_constraint(q_base_i, s_i)
        self.add_com_cost()

    def solve_optimization_problem(self):

        if not (self.solver_initialize):
            print(
                "Solver not initialized !! please either set the solver via set_solver or intialize the solver via initialize_solver"
            )
            return

        self.solver.minimize(self.cost_function)
        print("Solving")
        try:
            self.sol = self.solver.solve()
            print(self.solver.debug.value)
            print("Solved")
        except:
            print("WARNING NOT SOLVED PROBLEM DEBUG OUTPUT:")
            print(self.solver.debug.value)
            exit()

    def add_density_constraint(self, density_values):

        if not (self.solver_initialize):
            print(
                "Solver not initialized !! please either set the solver via set_solver or intialize the solver via initialize_solver"
            )
            return

        multiplication = 1.0
        for item in density_values:
            for i in range(self.numberOfParameter):
                multiplication = multiplication * (self.density_var[i] - item)
        self.cost_function += multiplication == 0.0

    def get_solutions(self):
        self.s_opt_all = self.sol.value(self.s)
        self.quat_b_opt = self.sol.value(self.quat_pose_b)
        self.lenght_optimum = self.sol.value(self.lengths_multipliers)
        self.density_optimum = self.sol.value(self.density_var)
        self.density_optimum_vector = (
            ergoCubConfiguration.CreateTotalHardwareParameterVector(
                self.partsList, self.density_optimum
            )
        )
        self.length_optimum_vector = (
            ergoCubConfiguration.CreateTotalHardwareParameterVector(
                self.partsList, self.lenght_optimum
            )
        )

    def pritty_print(self, name, value, color_custom="\033[94m"):
        BOLD = "\033[1m"
        END_COLOR = "\033[0m"
        print(BOLD, color_custom, name, END_COLOR, value)

    def print_solutions(self):
        END_COLOR = "\030"
        w_H_b_left_foot = self.H_left_foot(
            np.eye(4),
            np.zeros(self.N_DoF),
            self.density_optimum_vector,
            self.length_optimum_vector,
        )
        w_H_torso_num_zero = self.w_H_torso(
            np.eye(4),
            np.zeros(self.N_DoF),
            self.density_optimum_vector,
            self.length_optimum_vector,
        )
        H_height_num_temp = self.H_height(
            np.eye(4),
            np.zeros(self.N_DoF),
            self.density_optimum_vector,
            self.length_optimum_vector,
        )
        H_heigth_num = cs.mtimes(cs.inv(w_H_b_left_foot), H_height_num_temp)
        height_pose_num = H_heigth_num[2, 3] + 0.12
        w_H_b_zero = cs.mtimes(cs.inv(w_H_b_left_foot), w_H_torso_num_zero)
        CoM_pos = self.CoM(
            w_H_b_zero,
            np.zeros(self.N_DoF),
            self.density_optimum_vector,
            self.length_optimum_vector,
        )
        print(
            "-------------------------------------------------------------------------------------"
        )
        print(
            "-------------------------------------------------------------------------------------"
        )
        self.pritty_print("SOLUTION AND COST FUNCTION ERRORS", " ")
        self.pritty_print(
            "Lenght Optimum Vector: ", self.length_optimum_vector, END_COLOR
        )
        self.pritty_print(
            "Density Optimum Vector: ", self.density_optimum_vector, END_COLOR
        )
        self.pritty_print("Density Original :", self.density_vector, END_COLOR)
        self.pritty_print("Robot Height: ", height_pose_num, END_COLOR)
        self.pritty_print("CoM Pose: ", CoM_pos, END_COLOR)
        print(
            "-------------------------------------------------------------------------------------"
        )
        print(
            "-------------------------------------------------------------------------------------"
        )

        for k in range(self.number_of_points):
            if self.number_of_points > 1:
                s_opt = self.s_opt_all[:, k]
                w_H_b_num = self.H_b_fun(self.quat_b_opt[:, k])
                q_base_opt = self.quat_pose_b[:, k]
            else:
                s_opt = self.s_opt_all
                w_H_b_num = self.H_b_fun(self.quat_b_opt)
                q_base_opt = self.quat_pose_b
            w_H_rHand_num = self.H_right_hand(
                w_H_b_num,
                s_opt,
                self.density_optimum_vector,
                self.length_optimum_vector,
            )
            w_H_lHand_num = self.H_left_hand(
                w_H_b_num,
                s_opt,
                self.density_optimum_vector,
                self.length_optimum_vector,
            )
            w_H_lFoot_num = self.H_left_foot(
                w_H_b_num,
                s_opt,
                self.density_optimum_vector,
                self.length_optimum_vector,
            )
            w_H_rFoot_num = self.H_right_foot(
                w_H_b_num,
                s_opt,
                self.density_optimum_vector,
                self.length_optimum_vector,
            )
            error_right_hand_heights = w_H_rHand_num[2, 3] - self.height_hands_start[k]
            error_left_hand_heights = w_H_lHand_num[2, 3] - self.height_hands_start[k]
            error_heights = [
                float(error_right_hand_heights),
                float(error_left_hand_heights),
            ]
            print(
                "-------------------------------------------------------------------------------------"
            )
            print(
                "-------------------------------------------------------------------------------------"
            )
            print_string = "POSITION N." + str(k)
            self.pritty_print(print_string, " ")
            self.pritty_print("Joint Positions: ", s_opt * 180 / math.pi, END_COLOR)
            self.pritty_print("Quaternion: ", q_base_opt, END_COLOR)
            self.pritty_print("BasePose: ", w_H_b_num, END_COLOR)
            self.pritty_print(
                "Rot Error Right Hand: ",
                self.error_rot(w_H_rHand_num, self.w_H_rHand_des),
                END_COLOR,
            )
            self.pritty_print(
                "Rot Error Left Hand: ",
                self.error_rot(w_H_lHand_num, self.w_H_lHand_des),
                END_COLOR,
            )
            self.pritty_print(
                "Rot Error Left Foot: ",
                self.error_rot(w_H_lFoot_num, self.w_H_lFoot_des),
                END_COLOR,
            )
            self.pritty_print(
                "Rot Error Right Foot: ",
                self.error_rot(w_H_rFoot_num, self.w_H_rFoot_des),
                END_COLOR,
            )
            self.pritty_print(
                "SumSquare Quaternion: ", cs.sumsqr(q_base_opt[:4]), END_COLOR
            )
            self.pritty_print(
                "Height error hands Right and Left: ", error_heights, END_COLOR
            )
            self.pritty_print("Pos Left Foot: ", w_H_lFoot_num[:3, 3], END_COLOR)
            self.pritty_print("Pos Right Foot: ", w_H_rFoot_num[:3, 3], END_COLOR)
            print(
                "-------------------------------------------------------------------------------------"
            )
            print(
                "-------------------------------------------------------------------------------------"
            )
        print(
            "-------------------------------------------------------------------------------------"
        )
        print(
            "-------------------------------------------------------------------------------------"
        )
        print("VOLUMES AND MASSES ORIGINAL VS OPTIMUM:")
        for item in self.link_name_list:
            mass = self.kinDyn.get_link_mass(item)
            volume = self.kinDyn.get_link_volume(item)
            index = self.link_name_list.index(item)
            print(
                "-------------------------------------------------------------------------------------"
            )
            self.pritty_print(item, " ")
            self.pritty_print(
                "Original Mass: ", mass(self.density_optimum_vector[index], 1.0)
            )
            self.pritty_print(
                "Optimum Mass: ",
                mass(
                    self.density_optimum_vector[index],
                    self.length_optimum_vector[index],
                ),
            )
            self.pritty_print(
                "Original Volume: ", volume(self.density_optimum_vector[index], 1.0)
            )
            self.pritty_print(
                "Optimum Volume: ",
                volume(
                    self.density_optimum_vector[index],
                    self.length_optimum_vector[index],
                ),
            )
            print(
                "-------------------------------------------------------------------------------------"
            )

    def modify_urdf(self, urdf_path_modified):
        print("**************** MODIFY AND SAVING THE URDF***************")
        dummy_file = "no_gazebo_plugins.urdf"
        robot, gazebo_plugin_text = urdfModifiers.utils.load_robot_and_gazebo_plugins(
            self.urdf_path, dummy_file
        )

        for i in range(len(self.link_name_list)):
            link_name = self.link_name_list[i]
            link_modifier = eCubUtils.fromLinkCharToModifier(
                link_name, self.LinkCharacteristics[link_name], robot
            )
            modifications = {}
            modifications[urdfModifiers.utils.geometry.Modification.DIMENSION] = [
                self.length_optimum_vector[i],
                urdfModifiers.utils.geometry.Modification.MULTIPLIER,
            ]
            if self.densityParametrization:
                modifications[urdfModifiers.utils.geometry.Modification.DENSITY] = [
                    self.density_optimum_vector[i],
                    urdfModifiers.utils.geometry.Modification.MULTIPLIER,
                ]
            else:
                modifications[urdfModifiers.utils.geometry.Modification.DENSITY] = [
                    1.0,
                    urdfModifiers.utils.geometry.Modification.MULTIPLIER,
                ]

            link_modifier.modify(modifications)

        for i in range(len(self.joint_name_list)):
            joint_name = self.joint_name_list[i]
            if joint_name != "torso_roll":
                joint = self.get_joint_by_name(joint_name, self.kinDyn.robot_desc)
                parent_name = joint.parent
                joint_modifier = eCubUtils.FromJointCharToModifier(
                    joint_name, self.JointCharacteristics[joint_name], robot
                )
                index_parent = self.link_name_list.index(parent_name)
                modifications[urdfModifiers.utils.geometry.Modification.DIMENSION] = [
                    self.length_optimum_vector[index_parent],
                    urdfModifiers.utils.geometry.Modification.MULTIPLIER,
                ]
                modifications[urdfModifiers.utils.geometry.Modification.DENSITY] = [
                    1.0,
                    urdfModifiers.utils.geometry.Modification.MULTIPLIER,
                ]
                joint_modifier.modify(modifications)
        urdfModifiers.utils.write_urdf_to_file(
            robot, urdf_path_modified, gazebo_plugin_text
        )

    def set_solution(self, sol):
        self.sol = sol

    def get_joint_by_name(self, joint_name, robot):
        joint_list = [
            corresponding_joint
            for corresponding_joint in robot.joints
            if corresponding_joint.name == joint_name
        ]
        if len(joint_list) != 0:
            return joint_list[0]
        else:
            return None

    def compute_wrenches(self, H_b_i, s_i, density, lenght_mult):
        Jc = []
        for item in self.contact_frame:
            J = self.kinDyn.jacobian_fun(item)
            Jc = cs.vertcat(Jc, J(H_b_i, s_i, density, lenght_mult))
        M_param = np.array(self.M(H_b_i, s_i, density, lenght_mult))
        J_weigthed_mass = np.array(Jc @ np.linalg.inv(M_param) @ cs.transpose(Jc))
        N_A_first_part = np.array(cs.transpose(Jc) @ np.linalg.inv(J_weigthed_mass))
        N_A_second_part = np.array(Jc @ np.linalg.inv(M_param))
        N_A = np.array(np.eye(self.N_DoF + 6) - N_A_first_part @ N_A_second_part)
        g = np.array(self.g(H_b_i, s_i, density, lenght_mult))
        B = cs.vertcat(np.zeros([6, self.N_DoF]), np.eye(self.N_DoF))
        tau = np.array((cs.pinv(N_A @ B) @ N_A @ g))
        weight_tau = 1.0
        fc = np.array(J_weigthed_mass @ (Jc @ np.linalg.inv(M_param) @ (-B @ tau + g)))
        return fc
