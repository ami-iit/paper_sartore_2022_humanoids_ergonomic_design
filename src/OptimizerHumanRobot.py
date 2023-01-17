import numpy as np
from Optimizer import Optimizer
from OptimizerHuman import OptimizerHuman
from adam.casadi.computations import KinDynComputations
import casadi as cs
import utils as eCubUtils


class OptimizerHumanRobot:
    def __init__(
        self,
        optimizerRobot: Optimizer,
        optimizerHuman: OptimizerHuman,
        solver,
        box_urdf,
        box_base="base_link",
    ) -> None:

        self.optimizerRobot = optimizerRobot
        self.optimizerHuman = optimizerHuman
        self.solver = solver
        self.box_kindin = KinDynComputations(box_urdf, [], box_base)
        self.base_box = solver.variable(7, optimizerRobot.number_of_points)
        self.H_fun = eCubUtils.FromQuaternionToMatrix()
        self.number_of_points = optimizerRobot.number_of_points
        self.cost_function = 0

    def add_robot_to_optimizer(
        self,
        height_hands_star,
        target_densities,
        s_initial_tot,
        quat_b_initial_tot,
        compute_original,
    ):

        self.optimizerRobot.set_target_height_hands(height_hands_star)
        self.optimizerRobot.populating_optimization_problem()
        self.optimizerRobot.compute_original_density()
        self.optimizerRobot.set_references()
        if compute_original:
            self.solver.subject_to(
                self.optimizerRobot.lengths_multipliers
                == 1.0 * np.ones(self.optimizerRobot.numberOfParameter)
            )
     
    def add_human_to_optimizer(
        self, s_initial_human_tot, quat_b_initial_tot, height_hands_star
    ):

        self.optimizerHuman.set_initial_with_variable(
            s_initial_human_tot, quat_b_initial_tot
        )
        self.optimizerHuman.set_target_height_hands(height_hands_star)
        self.optimizerHuman.populating_optimization_problem()
        self.optimizerHuman.set_references()

    def compute_kin_din_box(self):

        box_frame_side_2_left = "side2_left_dummy_link"
        box_frame_side_2_right = "side2_right_dummy_link"
        box_frame_side_1_left = "side1_left_dummy_link"
        box_frame_side_1_right = "side1_right_dummy_link"

        self.M_box = self.box_kindin.mass_matrix_fun()

        self.J_side_1_left_box = self.box_kindin.jacobian_fun(box_frame_side_1_left)
        self.J_side_1_right_box = self.box_kindin.jacobian_fun(box_frame_side_1_right)

        self.J_side_2_left_box = self.box_kindin.jacobian_fun(box_frame_side_2_left)
        self.J_side_2_right_box = self.box_kindin.jacobian_fun(box_frame_side_2_right)

        self.g_box = self.box_kindin.gravity_term_fun()

    def compute_output_torque(self):
        tau = []
        wrenches = []
        for i in range(self.number_of_points):
            if self.number_of_points == 1:

                s_i_robot = self.optimizerRobot.s_opt_all
                q_base_i_robot = self.optimizerRobot.quat_b_opt
                H_b_i_robot = self.optimizerRobot.H_b_fun(q_base_i_robot)

                s_i_human = self.optimizerHuman.s_opt_all
                q_base_i_human = self.optimizerHuman.quat_b_opt
                H_b_i_human = self.optimizerHuman.H_b_fun(q_base_i_human)

                H_b_i_box = self.H_fun(self.box_pose_opt)
            else:

                s_i_robot = self.optimizerRobot.s_opt_all[:, i]
                q_base_i_robot = self.optimizerRobot.quat_b_opt[:, i]
                H_b_i_robot = self.optimizerRobot.H_b_fun(q_base_i_robot)

                s_i_human = self.optimizerHuman.s_opt_all[:, i]
                q_base_i_human = self.optimizerHuman.quat_b_opt[:, i]
                H_b_i_human = self.optimizerHuman.H_b_fun(q_base_i_human)

                H_b_i_box = self.H_fun(self.box_pose_opt[:, i])

            M_robot = self.optimizerRobot.M(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_optimum_vector,
                self.optimizerRobot.length_optimum_vector,
            )
            M_human = self.optimizerHuman.M(H_b_i_human, s_i_human)
            M_box = self.M_box(H_b_i_box, [])

            N_DoF_human = self.optimizerHuman.N_DOF
            N_DoF_robot = self.optimizerRobot.N_DoF

            g_human = self.optimizerHuman.g(H_b_i_human, s_i_human)
            g_robot = self.optimizerRobot.g(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_optimum_vector,
                self.optimizerRobot.length_optimum_vector,
            )
            g_box = self.g_box(H_b_i_box, [])

            J_right_hand_human = self.optimizerHuman.J_right_hand(
                H_b_i_human, s_i_human
            )
            J_left_hand_human = self.optimizerHuman.J_left_hand(H_b_i_human, s_i_human)
            J_left_foot_human = self.optimizerHuman.J_left(H_b_i_human, s_i_human)
            J_right_foot_human = self.optimizerHuman.J_right(H_b_i_human, s_i_human)

            J_right_hand_robot = self.optimizerRobot.J_right_hand(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_optimum_vector,
                self.optimizerRobot.length_optimum_vector,
            )
            J_left_hand_robot = self.optimizerRobot.J_left_hand(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_optimum_vector,
                self.optimizerRobot.length_optimum_vector,
            )
            J_right_foot_robot = self.optimizerRobot.J_right(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_optimum_vector,
                self.optimizerRobot.length_optimum_vector,
            )
            J_left_foot_robot = self.optimizerRobot.J_left(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_optimum_vector,
                self.optimizerRobot.length_optimum_vector,
            )

            J_side_1_left_box = self.J_side_1_left_box(H_b_i_box, [])
            J_side_1_right_box = self.J_side_1_right_box(H_b_i_box, [])

            J_side_2_left_box = self.J_side_2_left_box(H_b_i_box, [])
            J_side_2_right_box = self.J_side_2_right_box(H_b_i_box, [])

            J_side_1_box = cs.vertcat(
                J_side_1_left_box[:3, :], J_side_1_right_box[:3, :]
            )
            J_side_2_box = cs.vertcat(
                J_side_2_left_box[:3, :], J_side_2_right_box[:3, :]
            )

            g = np.array(cs.vertcat(g_robot, g_human, g_box))
            M = np.array(
                cs.vertcat(
                    cs.horzcat(
                        M_robot,
                        np.zeros([N_DoF_robot + 6, N_DoF_human + 6]),
                        np.zeros([N_DoF_robot + 6, 6]),
                    ),
                    cs.horzcat(
                        np.zeros([N_DoF_human + 6, N_DoF_robot + 6]),
                        M_human,
                        np.zeros([N_DoF_human + 6, 6]),
                    ),
                    cs.horzcat(
                        np.zeros([6, N_DoF_robot + 6]),
                        np.zeros([6, N_DoF_human + 6]),
                        M_box,
                    ),
                )
            )
            B_robot = cs.vertcat(
                np.zeros([6, self.optimizerRobot.N_DoF]),
                np.eye(self.optimizerRobot.N_DoF),
            )
            B_human = cs.vertcat(np.zeros([6, N_DoF_human]), np.eye(N_DoF_human))
            B = cs.vertcat(
                cs.horzcat(B_robot, np.zeros([N_DoF_robot + 6, N_DoF_human])),
                cs.horzcat(np.zeros([N_DoF_human + 6, N_DoF_robot]), B_human),
            )
            B = cs.vertcat(B, np.zeros([6, N_DoF_robot + N_DoF_human]))

            J_contact_feet_robot = cs.vertcat(J_left_foot_robot, J_right_foot_robot)
            J_contact_hand_robot = cs.vertcat(
                J_left_hand_robot[:3, :], J_right_hand_robot[:3, :]
            )
            J_contact_feet_human = cs.vertcat(J_left_foot_human, J_right_foot_human)
            J_contact_hands_human = cs.vertcat(
                J_left_hand_human[:3, :], J_right_hand_human[:3, :]
            )

            J_first_line = cs.horzcat(
                J_contact_feet_robot, np.zeros([12, N_DoF_human + 6]), np.zeros([12, 6])
            )
            J_second_line = cs.horzcat(
                np.zeros([12, N_DoF_robot + 6]), J_contact_feet_human, np.zeros([12, 6])
            )
            J_third_line = cs.horzcat(
                J_contact_hand_robot, np.zeros([6, N_DoF_human + 6]), -J_side_1_box
            )
            J_fourth_line = cs.horzcat(
                np.zeros([6, N_DoF_robot + 6]), J_contact_hands_human, -J_side_2_box
            )
            J = cs.vertcat(J_first_line, J_second_line, J_third_line, J_fourth_line)
            M_inv = np.array(cs.solve(M, np.eye(N_DoF_human + N_DoF_robot + 18)))

            J_weigthed_mass = np.array(J @ M_inv @ np.transpose(J))
            row, cols = J_weigthed_mass.shape
            J_weighted_mass_inv = np.array(cs.solve(J_weigthed_mass, np.eye(row, cols)))
            N_A_first_part = np.array(np.transpose(J) @ J_weighted_mass_inv)
            N_A_second_part = np.array(J @ M_inv)
            N_A = (
                np.eye(N_DoF_robot + 6 + N_DoF_human + 6 + 6)
                - N_A_first_part @ N_A_second_part
            )

            tau_i = np.linalg.pinv(N_A @ B) @ N_A @ g
            tau = cs.horzcat(tau, tau_i)

            wrenches_i = np.array(
                J_weighted_mass_inv @ (-J @ M_inv @ B @ tau_i + J @ M_inv @ g)
            )
            wrenches = cs.horzcat(wrenches, wrenches_i)

            w_H_left_foot_robot = np.array(
                self.optimizerRobot.H_left_foot(
                    H_b_i_robot,
                    s_i_robot,
                    self.optimizerRobot.density_optimum_vector,
                    self.optimizerRobot.length_optimum_vector,
                )
            )
            w_H_right_foot_robot = np.array(
                self.optimizerRobot.H_right_foot(
                    H_b_i_robot,
                    s_i_robot,
                    self.optimizerRobot.density_optimum_vector,
                    self.optimizerRobot.length_optimum_vector,
                )
            )
            w_H_right_hand_robot = np.array(
                self.optimizerRobot.H_right_hand(
                    H_b_i_robot,
                    s_i_robot,
                    self.optimizerRobot.density_optimum_vector,
                    self.optimizerRobot.length_optimum_vector,
                )
            )
            w_H_left_hand_robot = np.array(
                self.optimizerRobot.H_left_hand(
                    H_b_i_robot,
                    s_i_robot,
                    self.optimizerRobot.density_optimum_vector,
                    self.optimizerRobot.length_optimum_vector,
                )
            )

            w_H_left_foot_human = np.array(
                self.optimizerHuman.H_left_foot(H_b_i_human, s_i_human)
            )
            w_H_right_foot_human = np.array(
                self.optimizerHuman.H_right_foot(H_b_i_human, s_i_human)
            )
            w_H_left_hand_human = np.array(
                self.optimizerHuman.H_left_hand(H_b_i_human, s_i_human)
            )
            w_H_right_hand_human = np.array(
                self.optimizerHuman.H_right_hand(H_b_i_human, s_i_human)
            )

        return tau, wrenches

    def local_wrenches_array(self, f, frame_orientation):
        X_c = np.zeros(6)
        X_c[:3, :3] = frame_orientation
        X_c[3:, 3:] = frame_orientation

        f_local = np.transpose(X_c) @ f

        return f_local

    def minimum_torque_cost(self):
        self.compute_kin_din_box()
        for i in range(self.number_of_points):
            s_i_robot = self.optimizerRobot.s[:, i]
            q_base_i_robot = self.optimizerRobot.quat_pose_b[:, i]
            H_b_i_robot = self.optimizerRobot.H_b_fun(q_base_i_robot)

            s_i_human = self.optimizerHuman.s[:, i]
            q_base_i_human = self.optimizerHuman.quat_pose_b[:, i]
            H_b_i_human = self.optimizerHuman.H_b_fun(q_base_i_human)

            H_b_i_box = self.H_fun(self.base_box[:, i])

            M_robot = self.optimizerRobot.M(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_vector,
                self.optimizerRobot.lengths_multipliers_vector,
            )
            M_human = self.optimizerHuman.M(H_b_i_human, s_i_human)
            M_box = self.M_box(H_b_i_box, [])

            N_DoF_human = self.optimizerHuman.N_DOF
            N_DoF_robot = self.optimizerRobot.N_DoF

            g_human = self.optimizerHuman.g(H_b_i_human, s_i_human)
            g_robot = self.optimizerRobot.g(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_vector,
                self.optimizerRobot.lengths_multipliers_vector,
            )
            g_box = self.g_box(H_b_i_box, [])

            J_right_hand_human = self.optimizerHuman.J_right_hand(
                H_b_i_human, s_i_human
            )
            J_left_hand_human = self.optimizerHuman.J_left_hand(H_b_i_human, s_i_human)
            J_left_foot_human = self.optimizerHuman.J_left(H_b_i_human, s_i_human)
            J_right_foot_human = self.optimizerHuman.J_right(H_b_i_human, s_i_human)

            J_right_hand_robot = self.optimizerRobot.J_right_hand(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_vector,
                self.optimizerRobot.lengths_multipliers_vector,
            )
            J_left_hand_robot = self.optimizerRobot.J_left_hand(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_vector,
                self.optimizerRobot.lengths_multipliers_vector,
            )
            J_right_foot_robot = self.optimizerRobot.J_right(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_vector,
                self.optimizerRobot.lengths_multipliers_vector,
            )
            J_left_foot_robot = self.optimizerRobot.J_left(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_vector,
                self.optimizerRobot.lengths_multipliers_vector,
            )

            J_side_1_left_box = self.J_side_1_left_box(H_b_i_box, [])
            J_side_1_right_box = self.J_side_1_right_box(H_b_i_box, [])

            J_side_2_left_box = self.J_side_2_left_box(H_b_i_box, [])
            J_side_2_right_box = self.J_side_2_right_box(H_b_i_box, [])

            J_side_1_box = cs.vertcat(
                J_side_1_left_box[:3, :], J_side_1_right_box[:3, :]
            )
            J_side_2_box = cs.vertcat(
                J_side_2_left_box[:3, :], J_side_2_right_box[:3, :]
            )

            g = cs.vertcat(g_robot, g_human, g_box)
            M = cs.vertcat(
                cs.horzcat(
                    M_robot,
                    np.zeros([N_DoF_robot + 6, N_DoF_human + 6]),
                    np.zeros([N_DoF_robot + 6, 6]),
                ),
                cs.horzcat(
                    np.zeros([N_DoF_human + 6, N_DoF_robot + 6]),
                    M_human,
                    np.zeros([N_DoF_human + 6, 6]),
                ),
                cs.horzcat(
                    np.zeros([6, N_DoF_robot + 6]),
                    np.zeros([6, N_DoF_human + 6]),
                    M_box,
                ),
            )
            B = cs.vertcat(
                cs.horzcat(
                    self.optimizerRobot.B, np.zeros([N_DoF_robot + 6, N_DoF_human])
                ),
                cs.horzcat(
                    np.zeros([N_DoF_human + 6, N_DoF_robot]), self.optimizerHuman.B
                ),
            )
            B = cs.vertcat(B, np.zeros([6, N_DoF_robot + N_DoF_human]))

            J_contact_feet_robot = cs.vertcat(J_left_foot_robot, J_right_foot_robot)
            J_contact_hand_robot = cs.vertcat(
                J_left_hand_robot[:3, :], J_right_hand_robot[:3, :]
            )
            J_contact_feet_human = cs.vertcat(J_left_foot_human, J_right_foot_human)
            J_contact_hands_human = cs.vertcat(
                J_left_hand_human[:3, :], J_right_hand_human[:3, :]
            )

            J_first_line = cs.horzcat(
                J_contact_feet_robot, np.zeros([12, N_DoF_human + 6]), np.zeros([12, 6])
            )
            J_second_line = cs.horzcat(
                np.zeros([12, N_DoF_robot + 6]), J_contact_feet_human, np.zeros([12, 6])
            )
            J_third_line = cs.horzcat(
                J_contact_hand_robot, np.zeros([6, N_DoF_human + 6]), -J_side_1_box
            )
            J_fourth_line = cs.horzcat(
                np.zeros([6, N_DoF_robot + 6]), J_contact_hands_human, -J_side_2_box
            )
            J = cs.vertcat(J_first_line, J_second_line, J_third_line, J_fourth_line)
            M_inv = cs.solve(M, cs.MX.eye(M.size1()))

            J_weigthed_mass = J @ M_inv @ cs.transpose(J)
            J_weighted_mass_inv = cs.solve(
                J_weigthed_mass, cs.MX.eye(J_weigthed_mass.size1())
            )
            N_A_first_part = cs.transpose(J) @ J_weighted_mass_inv
            N_A_second_part = J @ M_inv
            N_A = (
                cs.MX.eye(N_DoF_robot + 6 + N_DoF_human + 6 + 6)
                - N_A_first_part @ N_A_second_part
            )

            tau = cs.pinv(N_A @ B) @ N_A @ g
            tau_human = tau[6 + N_DoF_robot :]

            tau_robot = tau[: 6 + N_DoF_robot]
            print(N_DoF_robot)
            cost_function_tau_robot = cs.sumsqr(tau_robot)  # @tau_robot
            cost_function_tau = cs.sumsqr(tau_human)
            self.cost_function += 1000000 * 100000 * cost_function_tau
            self.cost_function += 100000 * cost_function_tau_robot

            wrenches = J_weighted_mass_inv @ (-J @ M_inv @ B @ tau + J @ M_inv @ g)
            ## Transf Matrix
            w_H_left_foot_robot = self.optimizerRobot.H_left_foot(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_vector,
                self.optimizerRobot.lengths_multipliers_vector,
            )
            w_H_right_foot_robot = self.optimizerRobot.H_right_foot(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_vector,
                self.optimizerRobot.lengths_multipliers_vector,
            )
            w_H_right_hand_robot = self.optimizerRobot.H_right_hand(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_vector,
                self.optimizerRobot.lengths_multipliers_vector,
            )
            w_H_left_hand_robot = self.optimizerRobot.H_left_hand(
                H_b_i_robot,
                s_i_robot,
                self.optimizerRobot.density_vector,
                self.optimizerRobot.lengths_multipliers_vector,
            )

            w_H_left_foot_human = self.optimizerHuman.H_left_foot(
                H_b_i_human, s_i_human
            )
            w_H_right_foot_human = self.optimizerHuman.H_right_foot(
                H_b_i_human, s_i_human
            )
            w_H_left_hand_human = self.optimizerHuman.H_left_hand(
                H_b_i_human, s_i_human
            )
            w_H_right_hand_human = self.optimizerHuman.H_right_hand(
                H_b_i_human, s_i_human
            )

            # Wrenches Constraint
            static_friction = 1 / 10
            torsional_friction = 1 / 75
            wrenches_feet_robot = wrenches[:12]
            wrenches_feet_human = wrenches[12:24]
            # the hands have only the linear part !!!
            wrenches_hands_robot = wrenches[24:30]
            wrenches_hands_human = wrenches[30:]

            wrench_foot_robot_i_local = cs.vertcat(
                self.get_local_wrench(
                    wrenches_feet_robot[:6], w_H_left_foot_robot[:3, :3]
                ),
                self.get_local_wrench(
                    wrenches_feet_robot[6:], w_H_right_foot_robot[:3, :3]
                ),
            )

            wrench_hand_robot_temp = cs.vertcat(
                wrenches_hands_robot[:3],
                np.zeros([3, 1]),
                wrenches_hands_robot[3:],
                np.zeros([3, 1]),
            )
            wrench_hand_robot_i_local = cs.vertcat(
                self.get_local_wrench(
                    wrench_hand_robot_temp[:6], w_H_left_hand_robot[:3, :3]
                ),
                self.get_local_wrench(
                    wrench_hand_robot_temp[6:], w_H_right_hand_robot[:3, :3]
                ),
            )
            wrench_foot_human_i_local = cs.vertcat(
                self.get_local_wrench(
                    wrenches_feet_human[:6], w_H_left_foot_human[:3, :3]
                ),
                self.get_local_wrench(
                    wrenches_feet_human[6:], w_H_right_foot_human[:3, :3]
                ),
            )

            wrench_hand_human_temp = cs.vertcat(
                wrenches_hands_human[:3],
                np.zeros([3, 1]),
                wrenches_hands_human[3:],
                np.zeros([3, 1]),
            )
            wrench_hand_human_i_local = cs.vertcat(
                self.get_local_wrench(
                    wrench_hand_human_temp[:6], w_H_left_hand_human[:3, :3]
                ),
                self.get_local_wrench(
                    wrench_hand_human_temp[6:], w_H_right_hand_human[:3, :3]
                ),
            )

            cost_function_hands = (
                cs.sumsqr(wrench_hand_human_i_local[:3])
                + cs.sumsqr(wrench_hand_human_i_local[6:9])
                + cs.sumsqr(
                    wrench_hand_robot_i_local[:3] + wrench_hand_robot_i_local[6:9]
                )
            )
            cost_function_wrenches_human = cs.sumsqr(
                wrench_hand_human_i_local[:3] - wrench_hand_human_i_local[6:9]
            )
            cost_function_wrenches_robot = cs.sumsqr(
                wrench_hand_robot_i_local[:3] - wrench_hand_robot_i_local[6:9]
            )

            cost_function_cop_left = cs.sumsqr(
                wrench_foot_robot_i_local[4] / wrench_foot_robot_i_local[2]
            )
            cost_function_cop_left += cs.sumsqr(
                wrench_foot_robot_i_local[3] / wrench_foot_robot_i_local[2]
            )
            cost_function_cop_right = cs.sumsqr(
                wrench_foot_robot_i_local[10] / wrench_foot_robot_i_local[8]
            )
            cost_function_cop_right += cs.sumsqr(
                wrench_foot_robot_i_local[9] / wrench_foot_robot_i_local[8]
            )
            cost_function_cop_left += cs.sumsqr(
                wrench_foot_human_i_local[4] / wrench_foot_human_i_local[2]
            )
            cost_function_cop_left += cs.sumsqr(
                wrench_foot_human_i_local[3] / wrench_foot_human_i_local[2]
            )
            cost_function_cop_right += cs.sumsqr(
                wrench_foot_human_i_local[10] / wrench_foot_human_i_local[8]
            )
            cost_function_cop_right += cs.sumsqr(
                wrench_foot_human_i_local[9] / wrench_foot_human_i_local[8]
            )
            self.cost_function += cost_function_cop_left + cost_function_cop_right

            self.cost_function += (
                cost_function_wrenches_human + cost_function_wrenches_robot
            )

    def get_local_wrench(self, f, frame_orientation):

        X_c = cs.MX.zeros(6, 6)
        X_c[:3, :3] = frame_orientation
        X_c[3:, 3:] = frame_orientation

        f_local = cs.transpose(X_c) @ f

        return f_local

    def get_local_wrench_with_position(self, f, kinDin, frame_name, H_b, s_i):
        H = kinDin.forward_kinematics_fun(frame_name)(H_b, s_i)
        f_local = self.get_local_wrench(f, H[:3, :3])
        X_c = np.eye(6)
        X_c[:3, :3] = H[:3, :3]
        X_c[3:, 3:] = H[:3, :3]

        f_local = np.transpose(X_c) @ f
        return f_local

    def get_local_wrench_with_position_and_hardware(
        self, f, kinDin, frame_name, H_b, s_i, density, lenght
    ):
        H = kinDin.forward_kinematics_fun(frame_name)(H_b, s_i, density, lenght)
        X_c = np.eye(6)
        X_c[:3, :3] = H[:3, :3]
        X_c[3:, 3:] = H[:3, :3]

        f_local = np.transpose(X_c) @ f
        return f_local

    def add_torsional_friction_constraint(self, wrench, torsional_friction):
        self.solver.subject_to(wrench[5] - torsional_friction * wrench[2] < 0)
        self.solver.subject_to(-wrench[5] - torsional_friction * wrench[2] < 0)

    def add_static_friction_constraint(self, wrench, static_friction):
        cost_function_static_left = 1000000 * cs.sumsqr(
            wrench[0] - static_friction * wrench[2]
        )
        cost_function_static_left_x = 100000 * cs.sumsqr(
            wrench[1] - static_friction * wrench[2]
        )
        self.cost_function += cost_function_static_left
        self.cost_function += cost_function_static_left_x
    
    def add_box_constraint(self, height_star):

        H_base = self.box_kindin.forward_kinematics_fun("base_link")

        quat_base_star = [
            9.99995289e-01,
            -8.99139270e-06,
            3.66289281e-07,
            3.06947462e-03,
            -4.16279437e-02,
            -1.86378435e-02,
            8.00000000e-01,
        ]
        H_base_star = self.H_fun(quat_base_star)
        for i in range(self.optimizerRobot.number_of_points):
            print("setting constraint on object height", height_star[i])
            H_base_variable = self.H_fun(self.base_box[:, i])
            H_base_param = H_base(H_base_variable, [])
            theta_error = eCubUtils.zAxisAngle()
            error_rot, error_pos = eCubUtils.TransfMatrixError()
            self.solver.subject_to(H_base_param[2, 3] == height_star[i])
            self.solver.subject_to(cs.sumsqr(self.base_box[:4, i]) == 1.0)
            self.solver.subject_to(self.base_box[:4, i] == quat_base_star[:4])

    def add_contact(self):

        box_frame_side_2_left = "side2_left_dummy_link"
        box_frame_side_2_right = "side2_right_dummy_link"
        box_frame_side_1_left = "side1_left_dummy_link"
        box_frame_side_1_right = "side1_right_dummy_link"

        human_frame_right = "RightHand"
        human_frame_left = "LeftHand"

        robot_frame_right = "l_hand_frame"
        robot_frame_left = "r_hand_frame"

        H_right_human = self.optimizerHuman.kinDyn.forward_kinematics_fun(
            human_frame_right
        )
        H_left_human = self.optimizerHuman.kinDyn.forward_kinematics_fun(
            human_frame_left
        )
        H_right_robot = self.optimizerRobot.kinDyn.forward_kinematics_fun(
            robot_frame_right
        )
        H_left_robot = self.optimizerRobot.kinDyn.forward_kinematics_fun(
            robot_frame_left
        )

        H_side_1_left_box = self.box_kindin.forward_kinematics_fun(
            box_frame_side_1_left
        )
        H_side_1_right_box = self.box_kindin.forward_kinematics_fun(
            box_frame_side_1_right
        )
        H_side_2_left_box = self.box_kindin.forward_kinematics_fun(
            box_frame_side_2_left
        )
        H_side_2_right_box = self.box_kindin.forward_kinematics_fun(
            box_frame_side_2_right
        )

        s_human = self.optimizerHuman.s
        quat_human = self.optimizerHuman.quat_pose_b

        s_robot = self.optimizerRobot.s
        quat_robot = self.optimizerRobot.quat_pose_b

        for i in range(self.optimizerRobot.number_of_points):

            H_b_human_i = self.H_fun(quat_human[:, i])
            H_b_robot_i = self.H_fun(quat_robot[:, i])
            s_human_i = s_human[:, i]
            s_robot_i = s_robot[:, i]
            H_b_box_i = self.H_fun(self.base_box[:, i])
            H_right_human_param = H_right_human(H_b_human_i, s_human_i)
            H_left_human_param = H_left_human(H_b_human_i, s_human_i)

            H_right_robot_param = H_right_robot(
                H_b_robot_i,
                s_robot_i,
                self.optimizerRobot.density,
                self.optimizerRobot.lengths_multipliers_vector,
            )
            H_left_robot_param = H_left_robot(
                H_b_robot_i,
                s_robot_i,
                self.optimizerRobot.density,
                self.optimizerRobot.lengths_multipliers_vector,
            )

            H_side_1_left_box_param = H_side_1_left_box(H_b_box_i, [])
            H_side_1_right_box_param = H_side_1_right_box(H_b_box_i, [])

            H_side_2_left_box_param = H_side_2_left_box(H_b_box_i, [])
            H_side_2_right_box_param = H_side_2_right_box(H_b_box_i, [])

            ## Contact Position are one equal to the other
            self.solver.subject_to(
                H_right_robot_param[:3, 3] == H_side_1_left_box_param[:3, 3]
            )
            self.solver.subject_to(
                H_left_robot_param[:3, 3] == H_side_1_right_box_param[:3, 3]
            )
            self.solver.subject_to(
                H_right_human_param[:3, 3] == H_side_2_left_box_param[:3, 3]
            )
            self.solver.subject_to(
                H_left_human_param[:3, 3] == H_side_2_right_box_param[:3, 3]
            )

    def solve_optimization_problem(self, urdf_path_modified):
        self.cost_function += self.optimizerHuman.cost_function
        self.cost_function += self.optimizerRobot.cost_function
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

        self.optimizerHuman.set_solutions(self.sol)
        self.optimizerRobot.set_solution(self.sol)
        self.box_pose_opt = self.sol.value(self.base_box)
        self.optimizerRobot.get_solutions()
        self.optimizerRobot.modify_urdf(urdf_path_modified)
