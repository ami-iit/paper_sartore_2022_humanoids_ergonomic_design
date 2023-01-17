from os import link
from adam.core import link_parametric
import casadi as cs
import math


def get_link_join_char(linkPartName):
    links_characteristics = []
    joint_characteristics = []
    link_name_list = []
    if linkPartName == "RightArm":
        link_name_list = ["r_upper_arm", "r_forearm"]
        joint_name_list = ["r_elbow", "r_wrist_pitch"]
        links_characteristics = {
            link_name_list[0]: link_parametric.LinkCharacteristics(0.022),
            link_name_list[1]: link_parametric.LinkCharacteristics(0.03904),
        }
        joint_characteristics = {
            joint_name_list[0]: link_parametric.JointCharacteristics(0.0344),
            joint_name_list[1]: link_parametric.JointCharacteristics(0.0506),
        }
        return (
            link_name_list,
            joint_name_list,
            links_characteristics,
            joint_characteristics,
        )
    if linkPartName == "LeftArm":
        link_name_list = ["l_upper_arm", "l_forearm"]
        joint_name_list = ["l_elbow", "l_wrist_pitch"]
        links_characteristics = {
            link_name_list[0]: link_parametric.LinkCharacteristics(0.022),
            link_name_list[1]: link_parametric.LinkCharacteristics(0.03904),
        }
        joint_characteristics = {
            joint_name_list[0]: link_parametric.JointCharacteristics(0.0344),
            joint_name_list[1]: link_parametric.JointCharacteristics(0.0506),
        }
        return (
            link_name_list,
            joint_name_list,
            links_characteristics,
            joint_characteristics,
        )
    if linkPartName == "RightLeg":
        link_name_list = ["r_hip_3", "r_lower_leg"]
        joint_name_list = ["r_hip_yaw", "r_ankle_pitch"]
        links_characteristics = {
            link_name_list[0]: link_parametric.LinkCharacteristics(0.058),
            link_name_list[1]: link_parametric.LinkCharacteristics(-0.03),
        }
        joint_characteristics = {
            joint_name_list[0]: link_parametric.JointCharacteristics(0.1451),
            joint_name_list[1]: link_parametric.JointCharacteristics(-0.055989),
        }
        return (
            link_name_list,
            joint_name_list,
            links_characteristics,
            joint_characteristics,
        )
    if linkPartName == "LeftLeg":
        link_name_list = ["l_hip_3", "l_lower_leg"]
        joint_name_list = ["l_hip_yaw", "l_ankle_pitch"]
        links_characteristics = {
            link_name_list[0]: link_parametric.LinkCharacteristics(0.058),
            link_name_list[1]: link_parametric.LinkCharacteristics(-0.03),
        }
        joint_characteristics = {
            joint_name_list[0]: link_parametric.JointCharacteristics(0.1451),
            joint_name_list[1]: link_parametric.JointCharacteristics(-0.055989),
        }
        return (
            link_name_list,
            joint_name_list,
            links_characteristics,
            joint_characteristics,
        )
    if linkPartName == "Torso":
        link_name_list = ["root_link", "torso_1", "torso_2", "chest"]
        joint_name_list = [
            "torso_pitch",
            "torso_yaw",
            "torso_roll",
            "r_hip_pitch",
            "l_hip_pitch",
            "r_shoulder_pitch",
            "l_shoulder_pitch",
            "neck_fixed_joint",
        ]
        commonLinkCharacteristic = link_parametric.LinkCharacteristics(
            0,
            link_parametric.Side.DEPTH,
            flip_direction=True,
            calculate_origin_from_dimension=False,
        )
        links_characteristics = {
            link_name_list[0]: commonLinkCharacteristic,
            link_name_list[1]: commonLinkCharacteristic,
            link_name_list[2]: commonLinkCharacteristic,
            link_name_list[3]: commonLinkCharacteristic,
        }
        # TODO check order of the variable input in the constructor, something seems fishy
        joint_characteristics = {
            joint_name_list[0]: link_parametric.JointCharacteristics(
                -0.078, flip_direction=False
            ),
            joint_name_list[1]: link_parametric.JointCharacteristics(
                -0.07113, flip_direction=False
            ),
            joint_name_list[2]: link_parametric.JointCharacteristics(
                0.0, take_half_length=True, flip_direction=True, modify_origin=False
            ),
            joint_name_list[3]: link_parametric.JointCharacteristics(
                0.0494, take_half_length=True
            ),
            joint_name_list[4]: link_parametric.JointCharacteristics(
                0.0494, take_half_length=True
            ),
            joint_name_list[5]: link_parametric.JointCharacteristics(
                0.0554, take_half_length=True, flip_direction=False
            ),
            joint_name_list[6]: link_parametric.JointCharacteristics(
                0.0554, take_half_length=True, flip_direction=False
            ),
            joint_name_list[7]: link_parametric.JointCharacteristics(
                0.0607, take_half_length=True, flip_direction=False
            ),
        }
        return (
            link_name_list,
            joint_name_list,
            links_characteristics,
            joint_characteristics,
        )


def NumberHardwareParameterForSymmetry(nameList):
    number_parameter = 0
    if "RightLeg" in nameList or "LeftLeg" in nameList:
        number_parameter += 2
    if "LeftArm" in nameList or "RightArm" in nameList:
        number_parameter += 2
    if "Torso" in nameList:
        number_parameter += 4  # Torso to do for now do not keep it

    return number_parameter


def CreateTotalHardwareParameterVector(linkNameList, length_variable):
    length_Vector = []
    index_start = 0
    if (
        "RightArm" in linkNameList and "LeftArm" in linkNameList
    ):  # For now we assume they are the first one always
        length_arms = length_variable[0:2]
        length_Vector = cs.vertcat(length_Vector, length_arms, length_arms)
        index_start = 2
    if "LeftLeg" in linkNameList and "RightLeg" in linkNameList:
        length_legs = length_variable[index_start : index_start + 2]
        length_Vector = cs.vertcat(length_Vector, length_legs, length_legs)
        index_start += 2
    if "Torso" in linkNameList:
        length_torso = length_variable[index_start : index_start + 4]
        length_Vector = cs.vertcat(length_Vector, length_torso)
    return length_Vector


def GetLinkListPart(linkPartName):
    link_name_list = []
    print(linkPartName)
    for item in linkPartName:
        print(item)
        link_name_list_temp = []
        if item == "Head":
            link_name_list_temp = ["neck_1", "neck_2", "neck_3"]
        if item == "RightArm":
            link_name_list_temp = [
                "r_shoulder_1",
                "r_shoulder_2",
                "r_shoulder_3",
                "r_upper_arm",
                "r_elbow_1",
                "r_forearm",
            ]
        if item == "RightHand":
            link_name_list_temp = ["r_wrist_1", "r_hand"]
        if item == "LeftArm":
            link_name_list_temp = [
                "l_shoulder_1",
                "l_shoulder_2",
                "l_shoulder_3",
                "l_upper_arm",
                "l_elbow_1",
                "l_forearm",
            ]
        if item == "LeftHand":
            link_name_list_temp = ["l_wrist_1", "l_hand"]
        if item == "RightLeg":
            link_name_list_temp = [
                "r_hip_1",
                "r_hip_2",
                "r_hip_3",
                "r_upper_leg",
                "r_lower_leg",
            ]
        if item == "LeftLeg":
            link_name_list_temp = [
                "l_hip_1",
                "l_hip_2",
                "l_hip_3",
                "l_upper_leg",
                "l_lower_leg",
            ]
        if item == "Torso":
            link_name_list_temp = ["torso_1", "torso_2", "chest"]
        if item == "LeftFoot":
            link_name_list_temp = [
                "l_ankle_1",
                "l_ankle_2",
                "l_foot_front",
                "l_foot_rear",
            ]
        if item == "RightFoot":
            link_name_list_temp = [
                "r_ankle_1",
                "r_ankle_2",
                "r_foot_front",
                "r_foot_rear",
            ]
        link_name_list += link_name_list_temp
    return link_name_list


def get_initial_human_configuration(joints_name_list):
    s_init = {
        "jL5S1_rotx": 0,
        "jL5S1_roty": 0,
        "jL4L3_rotx": 0,
        "jL4L3_roty": 0,
        "jL1T12_rotx": 0,
        "jL1T12_roty": 0,
        "jT9T8_rotx": 0,
        "jT9T8_roty": 0,
        "jT9T8_rotz": 0,
        "jT1C7_rotx": 0,
        "jT1C7_roty": 0,
        "jT1C7_rotz": 0,
        "jC1Head_rotx": 0,
        "jC1Head_roty": 0,
        "jRightC7Shoulder_rotx": 0,
        "jRightShoulder_rotx": 0,
        "jRightShoulder_roty": 0,
        "jRightShoulder_rotz": 90,
        "jRightElbow_roty": 0,
        "jRightElbow_rotz": 0,
        "jRightWrist_rotx": 0,
        "jRightWrist_rotz": 0,
        "jLeftC7Shoulder_rotx": 0,
        "jLeftShoulder_rotx": 0,
        "jLeftShoulder_roty": 0,
        "jLeftShoulder_rotz": -90,
        "jLeftElbow_roty": 0,
        "jLeftElbow_rotz": 0,
        "jLeftWrist_rotx": 0,
        "jLeftWrist_rotz": 0,
        "jRightHip_rotx": 0,
        "jRightHip_roty": -25,
        "jRightHip_rotz": 0,
        "jRightKnee_roty": 50,
        "jRightKnee_rotz": 0,
        "jRightAnkle_rotx": 0,
        "jRightAnkle_roty": -25,
        "jRightAnkle_rotz": 0,
        "jLeftHip_rotx": 0,
        "jLeftHip_roty": -25,
        "jLeftHip_rotz": 0,
        "jLeftKnee_roty": 50,
        "jLeftKnee_rotz": 0,
        "jLeftAnkle_rotx": 0,
        "jLeftAnkle_roty": -25,
        "jLeftAnkle_rotz": 0,
    }
    s_init_out = [s_init[item] * math.pi / 180 for item in joints_name_list]
    return s_init_out
