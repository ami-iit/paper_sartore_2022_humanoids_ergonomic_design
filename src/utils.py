from os import link
import numpy as np
import casadi as cs
import math
from adam.geometry import utils as AdamUtils
from adam.core import link_parametric
import idyntree.bindings as iDynTree
from urdfModifiers.core.linkModifier import LinkModifier
from urdfModifiers.core.jointModifier import JointModifier
import urdfModifiers.geometry.geometry as urdfModifierGeometry


def FromQuaternionToMatrix():
    f_opts = (dict(jit=False, jit_options=dict(flags="-Ofast")),)
    # Quaternion variable
    H = cs.SX.eye(4)
    q = cs.SX.sym("q", 7)
    R = cs.SX.eye(3) + 2 * q[0] * cs.skew(q[1:4]) + 2 * cs.mpower(cs.skew(q[1:4]), 2)

    H[:3, :3] = R
    H[:3, 3] = q[4:7]
    H = cs.Function("H", [q], [H])
    return H


# f_opts = dict(jit=False, jit_options=dict(flags="-Ofast")),
# # Quaternion variable
# H = cs.SX.eye(4)
# q = cs.SX.sym("q", 7)
# quat_normalized = q[:4]/cs.norm_2(q[:4])
# R = cs.SX.eye(3) + 2*quat_normalized[0]*cs.skew(quat_normalized[1:4]) +2*cs.mpower(cs.skew(quat_normalized[1:4]),2)

# H[:3,:3] = R
# H[:3,3] = q[4:7]
# H = cs.Function("H", [q], [H])
# return H
# returns casadi function computing the rotation error and the position error as error_rot, error_pos
def TransfMatrixError():
    H = cs.SX.sym("H", 4, 4)
    H_des = cs.SX.sym("H_des", 4, 4)
    error_rot = cs.SX.ones(3)
    error_pos = cs.SX.ones(3)
    R = H[:3, :3]
    R_des = H_des[:3, :3]
    p = H[:3, 3]
    p_des = H_des[:3, 3]

    Temp = cs.mtimes(R, cs.transpose(R_des))
    error_rot = SkewVee(Temp)
    error_pos = p - p_des
    error_rot = cs.Function("error_rot", [H, H_des], [error_rot])
    error_pos = cs.Function("error_pos", [H, H_des], [error_pos])
    return error_rot, error_pos


def SkewVee(X):
    X_skew = 0.5 * (X - cs.transpose(X))
    x = cs.vertcat(-X_skew[1, 2], X_skew[0, 2], -X_skew[0, 1])
    return x


def ComputeHeight(kinDyn, height_frame, left_foot_frame):
    H_height = kinDyn.forward_kinematics_fun(height_frame)
    H_foot = kinDyn.forward_kinematics_fun(left_foot_frame)
    return cs.mtimes(cs.inv(H_foot), H_height)


def ComputeOriginalDensity(kinDyn, link_name):
    link_original = kinDyn.get_element_by_name(link_name, kinDyn.robot_desc)
    mass = link_original.inertial.mass
    volume = 0
    visual_obj = link_original.visuals[0]
    if visual_obj.geometry.box is not None:
        width = link_original.visuals[0].geometry.box.size[0]
        depth = link_original.visuals[0].geometry.box.size[2]
        height = link_original.visuals[0].geometry.box.size[1]
        volume = width * depth * height
    if visual_obj.geometry.cylinder is not None:
        length = link_original.visuals[0].geometry.cylinder.length
        radius = link_original.visuals[0].geometry.cylinder.radius
        volume = math.pi * radius ** 2 * length
    if visual_obj.geometry.sphere is not None:
        radius = link_original.visuals[0].geometry.sphere.radius
        volume = 4 * (math.pi * radius ** 3) / 3
    return mass / volume


def fromLinkCharToModifier(item, linkCharacteristic, robot):
    offset = linkCharacteristic.offset
    flip_direction = linkCharacteristic.flip_direction
    dimension = linkCharacteristic.dimension
    calculate_origin_from_dimension = linkCharacteristic.calculate_origin_from_dimension
    return LinkModifier.from_name(
        item,
        robot,
        offset,
        urdfModifierGeometry.Side.DEPTH,
        flip_direction,
        calculate_origin_from_dimension,
    )


def FromJointCharToModifier(item, jointCharacteristic, robot):
    offset = jointCharacteristic.offset
    take_half_length = jointCharacteristic.take_half_length
    flip_direction = jointCharacteristic.flip_direction
    return JointModifier.from_name(
        item, robot, offset, take_half_length, flip_direction
    )


def ComputeTotalMassLinkChain(
    linkNameList, kinDyn, density, lengths_multipliers_vector
):
    mass_tot = 0.0
    ParametricLinkList = kinDyn.link_name_list
    for item in linkNameList:
        if item in ParametricLinkList:
            mass_temp = kinDyn.get_link_mass(item)
            index = ParametricLinkList.index(item)
            mass_tot += mass_temp(density[index], lengths_multipliers_vector[index])
        else:
            mass_temp = kinDyn.get_element_by_name(
                item, kinDyn.robot_desc
            ).inertial.mass
            mass_tot = mass_tot + mass_temp
    return mass_tot


def zAxisAngle():
    H = cs.SX.sym("H", 4, 4)
    theta = cs.SX.sym("theta")
    theta = cs.dot([0, 0, 1], H[:3, 2]) - 1

    error = cs.Function("error", [H], [theta])
    return error


def GetIDynTreeTransfMatrix(s, H):

    N_DoF = len(s)
    s_idyntree = iDynTree.VectorDynSize(N_DoF)
    pos_iDynTree = iDynTree.Position()
    R_iDynTree = iDynTree.Rotation()

    for i in range(N_DoF):
        s_idyntree.setVal(i, s[i])

    R_iDynTree.FromPython(H[:3, :3])
    for i in range(3):
        pos_iDynTree.setVal(i, float(H[i, 3]))
        for j in range(3):
            R_iDynTree.setVal(j, i, float(H[j, i]))

    return s_idyntree, pos_iDynTree, R_iDynTree
