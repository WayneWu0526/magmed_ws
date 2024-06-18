import os

import numpy as np
import pybullet as p
# import matplotlib.pyplot as plt
import rospy
# from magmed_msgs.srv import SelfCollisionCheck, SelfCollisionCheckRequest, SelfCollisionCheckResponse
from magmed_msgs.srv import SelfCollisionCheck, SelfCollisionCheckResponse
# from magmed_msgs.srv import SelfCollisionCheck

CHECKTHRESHOLD = 0.01

class MultiPanda_bullet:
    def __init__(self, base_poses, base_orientations, stepsize=1e-3, realtime=0, GUI=False, debug=False,
                 Collision=False, collision_shape='mesh', **kwargs):
        self.t = 0.0
        self.stepsize = stepsize
        self.realtime = realtime

        self.control_mode = "torque"

        self.position_control_gain_p = [100.0] * 7
        self.position_control_gain_d = [40.0] * 7
        self.max_torque = [100] * 7

        self.GUI = GUI
        self.debug = debug

        self.name = 'MP'
        self.end_effector = False

        # connect pybullet
        if self.GUI:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)

        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=30, cameraPitch=-20,
                                     cameraTargetPosition=[0, 0, 0.5])

        p.resetSimulation()
        p.setTimeStep(self.stepsize)
        p.setRealTimeSimulation(self.realtime)
        p.setGravity(0, 0, 0)
        self.min_dist_arm_history = []
        # load models

        self.n_robot = len(base_poses)

        self.base_poses = base_poses
        self.base_orientations = base_orientations

        p.setAdditionalSearchPath(os.path.join(os.path.dirname(__file__), '..'))

        self.plane = p.loadURDF("plane/plane.urdf", useFixedBase=True)
        p.changeDynamics(self.plane, -1, restitution=.95)

        self.robot_colors = ['red','purple','green', 'blue']

        self.robots = []
        self.dofs = []
        self.end_effector_indices = []

        self.joints = []
        self.q_min = []
        self.q_max = []
        self.qdot_max = []
        self.target_pos = []
        self.target_torque = []

        self.activedofs = []

        for r_idx in range(self.n_robot):
            ri_base_pos = self.base_poses[r_idx]
            ri_base_ori = p.getQuaternionFromEuler([0, 0, self.base_orientations[r_idx]])
            robot_color = self.robot_colors[r_idx % 4]

            if collision_shape == 'capsule':
                self.robot_name = 'Diana7_Magnet.urdf'
            else:
               self.robot_name = f'Diana7_Magnet_{robot_color}.urdf'

            robot_i = p.loadURDF(f"Diana7_Magnet/{self.robot_name}",
                                 useFixedBase=True,
                                 flags=p.URDF_USE_SELF_COLLISION,
                                 basePosition=ri_base_pos,
                                 baseOrientation=ri_base_ori)

            ri_dof = p.getNumJoints(robot_i)
            activedof = 0
            joints = []
            for joint_idx in range(ri_dof):
                tmp_joint_info = p.getJointInfo(robot_i, joint_idx)
                if tmp_joint_info[2] != 4:
                    activedof += 1
                    joints.append(joint_idx)
                    self.q_min.append(tmp_joint_info[8])
                    self.q_max.append(tmp_joint_info[9])
                    self.qdot_max.append(tmp_joint_info[11])
                    self.target_pos.append((tmp_joint_info[8] + tmp_joint_info[9]) / 2.0)
                    self.target_torque.append(0.)

            self.robots.append(robot_i)
            self.dofs.append(ri_dof)
            self.activedofs.append(activedof)
            self.joints.append(joints)

        self.n_dof = np.array(self.activedofs, dtype=int).sum()

        self.range_angles = []
        # Add joint value slider
        if self.debug:
            for r_idx in range(self.n_robot):
                for j_idx in range(self.activedofs[r_idx]):
                    joint_idx = np.array(self.activedofs[:r_idx], dtype=int).sum() + j_idx
                    self.range_angles.append(p.addUserDebugParameter(f'R{r_idx + 1}_J{j_idx + 1}',
                                                                     rangeMin=self.q_min[joint_idx],
                                                                     rangeMax=self.q_max[joint_idx],
                                                                     startValue=self.target_pos[joint_idx]))

                # if not Collision:
                #     for r1_idx in range(self.n_robot):
                #         for r2_idx in range(r1_idx + 1, self.n_robot):
                #             for l1 in self.joints[r1_idx]:
                #                 for l2 in self.joints[r2_idx]:
                #                     p.setCollisionFilterPair(self.robots[r1_idx], self.robots[r2_idx], l1, l2, 0)
            if not Collision:
                for r1_idx in range(self.n_robot):
                    for l1 in self.joints[r1_idx]:
                        for l2 in self.joints[r1_idx]:
                            if l1 != l2:
                                p.setCollisionFilterPair(self.robots[r1_idx], self.robots[r1_idx], l1, l2, 0)

        self.reset()

    def all2sep(self, joint_idx):
        for r_idx in range(self.n_robot):
            if joint_idx < np.array(self.activedofs[:r_idx + 1], dtype=int).sum():
                break
        j_idx = joint_idx - np.array(self.activedofs[:r_idx], dtype=int).sum()
        return r_idx, self.joints[r_idx][j_idx]

    def sep2all(self, r_idx, j_idx):
        return np.array(self.activedofs[:r_idx], dtype=int).sum() + self.joints[r_idx].index(j_idx)

    def reset(self):
        self.t = 0.0
        self.control_mode = "torque"
        for j in range(self.n_dof):
            self.target_pos[j] = (self.q_min[j] + self.q_max[j]) / 2.0
            self.target_torque[j] = 0.0
            robot_idx, joint_idx = self.all2sep(j)
            p.resetJointState(self.robots[robot_idx], joint_idx, targetValue=self.target_pos[j])

        self.resetController()

    def step(self):
        self.t += self.stepsize
        p.stepSimulation()

    def resetController(self):
        for r_idx in range(self.n_robot):
            p.setJointMotorControlArray(bodyUniqueId=self.robots[r_idx],
                                        jointIndices=self.joints[r_idx],
                                        controlMode=p.VELOCITY_CONTROL,
                                        forces=[0. for i in range(self.activedofs[r_idx])])

    def setControlMode(self, mode):
        if mode == "position":
            self.control_mode = "position"
        elif mode == "torque":
            if self.control_mode != "torque":
                self.resetController()
            self.control_mode = "torque"
        else:
            raise Exception('wrong control mode')

    def reset2TargetPositions(self, target_pos):
        assert len(target_pos) == self.n_dof
        self.target_pos = target_pos
        for joint_idx in range(self.n_dof):
            r_idx, j_idx = self.all2sep(joint_idx)
            p.resetJointState(self.robots[r_idx], j_idx, target_pos[joint_idx])

    def getJointStates(self):
        joint_positions = []
        joint_velocities = []
        joint_torques = []

        for joint_idx in range(self.n_dof):
            r_idx, j_idx = self.all2sep(joint_idx)

            joint_state = p.getJointStates(self.robots[r_idx], j_idx)
            joint_positions.append(joint_state[0])
            joint_velocities.append(joint_state[1])
            joint_torques.append(joint_state[3])

        return joint_positions, joint_velocities, joint_torques

    def setTargetPositions(self, target_pos):
        raise NotImplementedError

    def getTargetPositionfromGUI(self):
        assert self.GUI and self.debug, "This function is only for GUI+debug mode."
        GUI_position = []
        for param_id in self.range_angles:
            GUI_position.append(p.readUserDebugParameter(param_id))
        return GUI_position

    

    def check_collision(self, pos, check_threshold=2, return_links=False):
        pos = np.asarray(pos)
        assert len(pos.shape) == 1, 'Only for single joint value'

        self.reset2TargetPositions(pos)
        p.stepSimulation()

        closest_points = []
        for r1_idx in range(self.n_robot):
            for r2_idx in range(r1_idx + 1, self.n_robot):
                closest_points += list(
                    p.getClosestPoints(bodyA=self.robots[r1_idx], bodyB=self.robots[r2_idx], distance=check_threshold))

        min_dist = 1e10
        col_links = [-100, -100]

        for closest_point in closest_points:
            if min_dist > closest_point[8]:
                min_dist = closest_point[8]
                col_links = closest_point[3:5]

        if return_links:
            return min_dist, col_links
        else:
            return min_dist

    def check_collision2(self, pos, check_threshold=0.001, return_links=False):
        
        flag = False # 无碰撞
        pos = np.asarray(pos)
        assert len(pos.shape) == 1, 'Only for single joint value'
        self.reset2TargetPositions(pos)
        p.stepSimulation()
        closest_points_arm = []
        closest_points_plane = []
        for r1_idx in range(self.n_robot):

            closest_points_arm += list(
                p.getClosestPoints(bodyA=self.robots[r1_idx], bodyB=self.robots[r1_idx], distance=2))
            #closest_points_plane += list(
              #  p.getClosestPoints(bodyA=self.robots[r1_idx], bodyB=self.plane, distance=check_threshold))
        min_dist_arm = 1e10
        col_links = [-100, -100]  #

        for closest_point in closest_points_arm:
            
            if closest_point[3] not in [8] and closest_point[4] not in [8]:
                 continue
            if closest_point[3] not in [5] and closest_point[4] not in [5]:
                 continue
            # if closest_point[3] == closest_point[4]:
            #     continue
            # if closest_point[3] not in [8, 5, 3, 2, 1] or closest_point[4] not in [8, 5, 3, 2, 1]:
            #     continue
            # if abs(closest_point[3] - closest_point[4]) == 1:
            #     continue
            # if closest_point[3] == closest_point[4]:
            #     continue
            # if closest_point[3] not in [8] and closest_point[4] not in [8]:
            #     continue
            # if closest_point[3] in [8, 5] and closest_point[4] in [8, 5]:
            #     continue
            if min_dist_arm > closest_point[8]:
                min_dist_arm = closest_point[8]
                col_links = closest_point[3:5]

        min_dist_plane = 1e10

        for closest_point in closest_points_plane:
            print(closest_point[8])
            if min_dist_plane > closest_point[8]:

                min_dist_plane = closest_point[8]
        #self.min_dist_arm_history.append(min_dist_arm)

        # 绘制实时图表

        #plt.pause(0.01)  # 暂停一小段时间来实现实时绘图
        # print(f'机械臂，末端执行器最小距离为{min_dist_arm:.3f}，对应关节为{col_links}')


        # if return_links:
        #     return min_dist_arm, col_links, min_dist_plane
        # else:
        #     return min_dist_arm, min_dist_plane
        
        if min_dist_arm < check_threshold:
            flag = True
        return flag


    


def collision_checker(req):
    
    # print(f'type(req.joints)=',type(req.joints.joints))
    # print(f'req.joints=',req.joints.joints)
    pos = req.joints.joints
    #print(f'pos:',pos)
    collision_flag = env.check_collision2(pos,CHECKTHRESHOLD)
    return SelfCollisionCheckResponse(collision_flag)
    

if __name__ == '__main__':

    base_poses = [[0, 0, 0]]
    rospy.init_node('collision_checker_service', anonymous=True)
    base_orientations = [-1.5708, 1.5708, 3.1415, 0]

    env = MultiPanda_bullet(base_poses, base_orientations, debug=True, GUI=False, Collision=False,
                            collision_shape='mesh')

    
    server = rospy.Service('magmed_modules/selfCollisionCheck', SelfCollisionCheck, collision_checker)
   
    rospy.spin()
