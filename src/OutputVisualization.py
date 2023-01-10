from selectors import EpollSelector
import idyntree.bindings as iDynTree
from adam.casadi.computations import KinDynComputations
import utils as eCubUtils
import numpy as np


class OutputVisualization:
    def __init__(self, OptimizerHelper) -> None:
        self.OptimizerHelper = OptimizerHelper
     
    def prepare_visualization(self):
        self.viz = iDynTree.Visualizer()
        vizOpt = iDynTree.VisualizerOptions()
        vizOpt.winWidth = 1500
        vizOpt.winHeight = 1500
        self.viz.init(vizOpt)

        self.env = self.viz.enviroment()
        self.env.setElementVisibility("floor_grid", True)
        self.env.setElementVisibility("world_frame", False)
        self.viz.setColorPalette("meshcat")
        self.env.setElementVisibility("world_frame", False)
        self.frames = self.viz.frames()
        cam = self.viz.camera()
        cam.setPosition(iDynTree.Position(0, 3, 1.2))
        self.viz.camera().animator().enableMouseControl(True)

    def add_box(self, urdf_path_box, model_name, root_link, quat_base, AddModel=True):
        if AddModel:
            mdlLoader = iDynTree.ModelLoader()
            mdlLoader.loadReducedModelFromFile(urdf_path_box, [], root_link)
            self.viz.addModel(mdlLoader.model(), model_name)
        H_b_fun = eCubUtils.FromQuaternionToMatrix()
        fake_joints = np.ones(3)

        H_b_vis = H_b_fun(quat_base)
        [s_fake, pos_base, R_base] = eCubUtils.GetIDynTreeTransfMatrix(
            fake_joints, H_b_vis
        )
        T_b = iDynTree.Transform()
        T_b.setPosition(pos_base)
        T_b.setRotation(R_base)
        T_b.setPosition(pos_base)

        self.viz.modelViz(model_name).setPositions(T_b, iDynTree.VectorDynSize(0))

    def add_and_update_model(
        self,
        model_name,
        urdf_path,
        joints_list,
        root_link,
        quat_base,
        joint_position,
        density_vector,
        lenght_vector,
        delta=0,
        ChangeColor=False,
    ):
        mdlLoader = iDynTree.ModelLoader()
        mdlLoader.loadReducedModelFromFile(urdf_path, joints_list, root_link)
        self.viz.addModel(mdlLoader.model(), model_name)

        H_b_fun = eCubUtils.FromQuaternionToMatrix()

        H_b_vis = H_b_fun(quat_base)
        H_b_vis[1, 3] = H_b_vis[1, 3] - delta
        w_H_lFoot_num = self.OptimizerHelper.H_left_foot(
            H_b_vis, joint_position, density_vector, lenght_vector
        )
        w_H_rFoot_num = self.OptimizerHelper.H_right_foot(
            H_b_vis, joint_position, density_vector, lenght_vector
        )

        w_H_lHand_num = self.OptimizerHelper.H_left_hand(
            H_b_vis, joint_position, density_vector, lenght_vector
        )
        w_H_rHand_num = self.OptimizerHelper.H_right_hand(
            H_b_vis, joint_position, density_vector, lenght_vector
        )

        CoM_new = self.OptimizerHelper.CoM(
            H_b_vis, joint_position, density_vector, lenght_vector
        )
        H_com = np.eye(5)
        T_b_left_foot = iDynTree.Transform()
        T_b_right_foot = iDynTree.Transform()
        T_b_left_hand = iDynTree.Transform()
        T_b_right_hand = iDynTree.Transform()

        T_b_com = iDynTree.Transform()
        for temp in range(3):
            H_com[temp, 3] = CoM_new[temp]

        [_, pos_CoM_iDynTree, R_CoM_iDynTree] = eCubUtils.GetIDynTreeTransfMatrix(
            joint_position, H_com
        )
        [
            _,
            pos_left_foot_iDynTree,
            R_left_foot_iDynTree,
        ] = eCubUtils.GetIDynTreeTransfMatrix(joint_position, w_H_lFoot_num)
        [
            _,
            pos_right_foot_iDynTree,
            R_right_foot_iDynTree,
        ] = eCubUtils.GetIDynTreeTransfMatrix(joint_position, w_H_rFoot_num)
        [
            _,
            pos_right_hand_iDynTree,
            R_right_hand_iDynTree,
        ] = eCubUtils.GetIDynTreeTransfMatrix(joint_position, w_H_rHand_num)
        [
            _,
            pos_left_hand_iDynTree,
            R_left_hand_iDynTree,
        ] = eCubUtils.GetIDynTreeTransfMatrix(joint_position, w_H_lHand_num)

        if ChangeColor:
            self.viz.modelViz(model_name).setModelColor(
                iDynTree.ColorViz(iDynTree.Vector4_FromPython([1, 0.2, 0.2, 0.2]))
            )

        T_b_com.setPosition(pos_CoM_iDynTree)
        T_b_com.setRotation(R_CoM_iDynTree)
        T_b_com.setPosition(pos_CoM_iDynTree)
        T_b_left_foot.setPosition(pos_left_foot_iDynTree)
        T_b_left_foot.setRotation(R_left_foot_iDynTree)
        T_b_left_foot.setPosition(pos_left_foot_iDynTree)
        T_b_right_foot.setPosition(pos_right_foot_iDynTree)
        T_b_right_foot.setRotation(R_right_foot_iDynTree)
        T_b_right_foot.setPosition(pos_right_foot_iDynTree)
        T_b_right_hand.setPosition(pos_right_hand_iDynTree)
        T_b_right_hand.setRotation(R_right_hand_iDynTree)
        T_b_right_hand.setPosition(pos_right_hand_iDynTree)
        T_b_left_hand.setPosition(pos_left_hand_iDynTree)
        T_b_left_hand.setRotation(R_left_hand_iDynTree)
        T_b_left_hand.setPosition(pos_left_hand_iDynTree)
        index_left_foot = 1
        index_right_foot = 1
        index_com = 1
        index_right_hand = 1
        index_left_hand = 1
        T_b = iDynTree.Transform()
        [s_idyntree, pos, R_iDynTree] = eCubUtils.GetIDynTreeTransfMatrix(
            joint_position, H_b_vis
        )
        T_b.setPosition(pos)
        T_b.setRotation(R_iDynTree)
        T_b.setPosition(pos)
        self.viz.modelViz(model_name).setPositions(T_b, s_idyntree)
        return (
            index_left_foot,
            index_right_foot,
            index_com,
            index_right_hand,
            index_left_hand,
        )

    def add_and_update_model_no_hardware(
        self,
        model_name,
        urdf_path,
        joints_list,
        root_link,
        quat_base,
        joint_position,
        ModelOptimizer,
        delta=0,
        ChangeColor=False,
    ):
        mdlLoader = iDynTree.ModelLoader()
        mdlLoader.loadReducedModelFromFile(urdf_path, joints_list, root_link)
        self.viz.addModel(mdlLoader.model(), model_name)

        H_b_fun = eCubUtils.FromQuaternionToMatrix()

        H_b_vis = H_b_fun(quat_base)
        H_b_vis[1, 3] = H_b_vis[1, 3] - delta
        w_H_lFoot_num = ModelOptimizer.H_left_foot(H_b_vis, joint_position)
        w_H_rFoot_num = ModelOptimizer.H_right_foot(H_b_vis, joint_position)
        CoM_new = ModelOptimizer.CoM(H_b_vis, joint_position)
        H_com = np.eye(5)
        T_b_left_foot = iDynTree.Transform()
        T_b_right_foot = iDynTree.Transform()
        T_b_com = iDynTree.Transform()
        for temp in range(3):
            H_com[temp, 3] = CoM_new[temp]

        [_, pos_CoM_iDynTree, R_CoM_iDynTree] = eCubUtils.GetIDynTreeTransfMatrix(
            joint_position, H_com
        )
        [
            _,
            pos_left_foot_iDynTree,
            R_left_foot_iDynTree,
        ] = eCubUtils.GetIDynTreeTransfMatrix(joint_position, w_H_lFoot_num)
        [
            _,
            pos_right_foot_iDynTree,
            R_right_foot_iDynTree,
        ] = eCubUtils.GetIDynTreeTransfMatrix(joint_position, w_H_rFoot_num)

        if ChangeColor:
            self.viz.modelViz(model_name).setModelColor(
                iDynTree.ColorViz(iDynTree.Vector4_FromPython([1, 0.2, 0.2, 0.2]))
            )

        T_b_com.setPosition(pos_CoM_iDynTree)
        T_b_com.setRotation(R_CoM_iDynTree)
        T_b_com.setPosition(pos_CoM_iDynTree)
        T_b_left_foot.setPosition(pos_left_foot_iDynTree)
        T_b_left_foot.setRotation(R_left_foot_iDynTree)
        T_b_left_foot.setPosition(pos_left_foot_iDynTree)
        T_b_right_foot.setPosition(pos_right_foot_iDynTree)
        T_b_right_foot.setRotation(R_right_foot_iDynTree)
        T_b_right_foot.setPosition(pos_right_foot_iDynTree)
        index_left_foot = 1
        index_right_foot = 1
        index_com = 1
        T_b = iDynTree.Transform()
        [s_idyntree, pos, R_iDynTree] = eCubUtils.GetIDynTreeTransfMatrix(
            joint_position, H_b_vis
        )
        T_b.setPosition(pos)
        T_b.setRotation(R_iDynTree)
        T_b.setPosition(pos)
        self.viz.modelViz(model_name).setPositions(T_b, s_idyntree)
        return index_left_foot, index_right_foot, index_com

    def update_model_no_hardware(
        self,
        Optimizer,
        model_name,
        quat_base,
        joint_position,
        index_left_foot,
        index_right_foot,
        index_com,
        delta=0,
    ):

        H_b_fun = eCubUtils.FromQuaternionToMatrix()
        H_b_vis = H_b_fun(quat_base)
        H_b_vis[1, 3] = H_b_vis[1, 3] - delta
        w_H_lFoot_num = Optimizer.H_left_foot(H_b_vis, joint_position)
        w_H_rFoot_num = Optimizer.H_right_foot(H_b_vis, joint_position)
        CoM_new = Optimizer.CoM(H_b_vis, joint_position)
        H_com = np.eye(5)
        T_b_left_foot = iDynTree.Transform()
        T_b_right_foot = iDynTree.Transform()
        T_b_com = iDynTree.Transform()
        for temp in range(3):
            H_com[temp, 3] = CoM_new[temp]

        [_, pos_CoM_iDynTree, R_CoM_iDynTree] = eCubUtils.GetIDynTreeTransfMatrix(
            joint_position, H_com
        )
        [
            _,
            pos_left_foot_iDynTree,
            R_left_foot_iDynTree,
        ] = eCubUtils.GetIDynTreeTransfMatrix(joint_position, w_H_lFoot_num)
        [
            _,
            pos_right_foot_iDynTree,
            R_right_foot_iDynTree,
        ] = eCubUtils.GetIDynTreeTransfMatrix(joint_position, w_H_rFoot_num)

        T_b_com.setPosition(pos_CoM_iDynTree)
        T_b_com.setRotation(R_CoM_iDynTree)
        T_b_com.setPosition(pos_CoM_iDynTree)
        T_b_left_foot.setPosition(pos_left_foot_iDynTree)
        T_b_left_foot.setRotation(R_left_foot_iDynTree)
        T_b_left_foot.setPosition(pos_left_foot_iDynTree)
        T_b_right_foot.setPosition(pos_right_foot_iDynTree)
        T_b_right_foot.setRotation(R_right_foot_iDynTree)
        T_b_right_foot.setPosition(pos_right_foot_iDynTree)
        T_b = iDynTree.Transform()
        [s_idyntree, pos, R_iDynTree] = eCubUtils.GetIDynTreeTransfMatrix(
            joint_position, H_b_vis
        )
        T_b.setPosition(pos)
        T_b.setRotation(R_iDynTree)
        T_b.setPosition(pos)
        self.viz.modelViz(model_name).setPositions(T_b, s_idyntree)

    def update_model(
        self,
        model_name,
        quat_base,
        joint_position,
        index_left_foot,
        index_right_foot,
        index_com,
        index_left_hand,
        index_right_hand,
        density_vector,
        lenght_vector,
        delta=0,
    ):

        H_b_fun = eCubUtils.FromQuaternionToMatrix()
        H_b_vis = H_b_fun(quat_base)
        H_b_vis[1, 3] = H_b_vis[1, 3] - delta
        w_H_lFoot_num = self.OptimizerHelper.H_left_foot(
            H_b_vis, joint_position, density_vector, lenght_vector
        )
        w_H_rFoot_num = self.OptimizerHelper.H_right_foot(
            H_b_vis, joint_position, density_vector, lenght_vector
        )
        w_H_lHand_num = self.OptimizerHelper.H_left_hand(
            H_b_vis, joint_position, density_vector, lenght_vector
        )
        w_H_rHand_num = self.OptimizerHelper.H_right_hand(
            H_b_vis, joint_position, density_vector, lenght_vector
        )

        CoM_new = self.OptimizerHelper.CoM(
            H_b_vis, joint_position, density_vector, lenght_vector
        )
        H_com = np.eye(5)
        T_b_left_foot = iDynTree.Transform()
        T_b_right_foot = iDynTree.Transform()
        T_b_com = iDynTree.Transform()
        T_b_left_hand = iDynTree.Transform()
        T_b_right_hand = iDynTree.Transform()
        for temp in range(3):
            H_com[temp, 3] = CoM_new[temp]

        [_, pos_CoM_iDynTree, R_CoM_iDynTree] = eCubUtils.GetIDynTreeTransfMatrix(
            joint_position, H_com
        )
        [
            _,
            pos_left_foot_iDynTree,
            R_left_foot_iDynTree,
        ] = eCubUtils.GetIDynTreeTransfMatrix(joint_position, w_H_lFoot_num)
        [
            _,
            pos_right_foot_iDynTree,
            R_right_foot_iDynTree,
        ] = eCubUtils.GetIDynTreeTransfMatrix(joint_position, w_H_rFoot_num)
        [
            _,
            pos_right_hand_iDynTree,
            R_right_hand_iDynTree,
        ] = eCubUtils.GetIDynTreeTransfMatrix(joint_position, w_H_rHand_num)
        [
            _,
            pos_left_hand_iDynTree,
            R_left_hand_iDynTree,
        ] = eCubUtils.GetIDynTreeTransfMatrix(joint_position, w_H_lHand_num)

        T_b_com.setPosition(pos_CoM_iDynTree)
        T_b_com.setRotation(R_CoM_iDynTree)
        T_b_com.setPosition(pos_CoM_iDynTree)
        T_b_left_foot.setPosition(pos_left_foot_iDynTree)
        T_b_left_foot.setRotation(R_left_foot_iDynTree)
        T_b_left_foot.setPosition(pos_left_foot_iDynTree)
        T_b_right_foot.setPosition(pos_right_foot_iDynTree)
        T_b_right_foot.setRotation(R_right_foot_iDynTree)
        T_b_right_foot.setPosition(pos_right_foot_iDynTree)
        T_b_right_hand.setPosition(pos_right_hand_iDynTree)
        T_b_right_hand.setRotation(R_right_hand_iDynTree)
        T_b_right_hand.setPosition(pos_right_hand_iDynTree)
        T_b_left_hand.setPosition(pos_left_hand_iDynTree)
        T_b_left_hand.setRotation(R_left_hand_iDynTree)
        T_b_left_hand.setPosition(pos_left_hand_iDynTree)
        T_b = iDynTree.Transform()
        [s_idyntree, pos, R_iDynTree] = eCubUtils.GetIDynTreeTransfMatrix(
            joint_position, H_b_vis
        )
        T_b.setPosition(pos)
        T_b.setRotation(R_iDynTree)
        T_b.setPosition(pos)
        self.viz.modelViz(model_name).setPositions(T_b, s_idyntree)

    def visualize_wrench(self, kind_din, frame_name, H_b, s_i, wrench, idx=None):
        T_c = kind_din.forward_kinematics_fun(frame_name)(H_b, s_i)
        T_c_idyn = iDynTree.Transform()
        T_c_idyn.fromHomogeneousTransform(iDynTree.Matrix4x4(T_c))
        wrench_new = np.divide(wrench, 200)
        force = iDynTree.Direction().FromPython(np.reshape(wrench_new[:3], 3))
        if idx is None:
            idx = self.viz.vectors().addVector(T_c_idyn.getPosition(), force)
        else:
            self.viz.vectors().updateVector(idx, T_c_idyn.getPosition(), force)

        return idx

    def visualize_wrench_param_model(
        self, kind_din, frame_name, H_b, s_i, l_m, density, wrench, idx=None
    ):
        T_c = kind_din.forward_kinematics_fun(frame_name)(H_b, s_i, density, l_m)
        T_c_idyn = iDynTree.Transform()
        wrench_new = np.divide(wrench, 200)
        T_c_idyn.fromHomogeneousTransform(iDynTree.Matrix4x4(T_c))
        force = iDynTree.Direction().FromPython(np.reshape(wrench_new[:3], 3))
        if idx is None:
            idx = self.viz.vectors().addVector(T_c_idyn.getPosition(), force)
        else:
            self.viz.vectors().updateVector(idx, T_c_idyn.getPosition(), force)
        return idx
