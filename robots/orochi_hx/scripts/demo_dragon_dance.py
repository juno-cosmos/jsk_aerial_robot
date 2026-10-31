#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)

# Copyright (c) 2025, DRAGON Laboratory, The University of Tokyo
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import math
import numpy as np

from orochi_hx.robot_interface import OrochiHxInterface
from aerial_robot_base.state_machine import *


# body shape of the dragon dance
#  all joints are on the sine curve on the cylinder: z = A * sin(2 * pi * r * theta / lambda) (cylindrical coordinates)
#  the sine curve is fixed in the world, thus the wave travels from head to tail as the robot goes around the circle
# NOTE: the relative rotation between female{i} and female{i+1} is Rz(joint_yaw{i}) * Rx(joint_roll{i+1}),
#       and each link extends along -x axis of female{i}.
class DragonDanceShape():
    def __init__(self, link_num = 4, link_length = 0.53053, radius = 1.25, wave_length = math.pi, \
                 direction = -1, baselink_index = 1):
        self.link_num = link_num
        self.link_length = link_length
        self.radius = radius # radius of the cylinder [m]
        self.wave_length = wave_length # wave length along the circle [m]
        self.direction = direction # 1: CCW, -1: CW
        self.baselink_index = baselink_index # fc is on female2

    def height(self, theta, amplitude):
        return amplitude * math.sin(2 * math.pi * self.radius * theta / self.wave_length)

    # return joint_yaw (link_num - 1), joint_roll (link_num), roll, pitch, yaw of the baselink, and the CoG position
    # in the cylindrical frame, where theta is the angle of the head
    def calc(self, theta, amplitude):
        n = self.link_num
        r = self.radius

        # joints on the cylinder, the distance between neighboring joints is the link length
        thetas = [theta]
        points = [np.array([r * math.cos(theta), r * math.sin(theta), self.height(theta, amplitude)])]
        for i in range(n):
            d_theta = 2 * math.asin(self.link_length / (2 * r))
            for _ in range(20):
                dz = self.height(thetas[-1] - self.direction * d_theta, amplitude) - points[-1][2]
                chord = math.sqrt(max(self.link_length ** 2 - dz ** 2, 1e-6))
                d_theta = 2 * math.asin(min(chord / (2 * r), 1.0))
            th = thetas[-1] - self.direction * d_theta
            thetas.append(th)
            points.append(np.array([r * math.cos(th), r * math.sin(th), self.height(th, amplitude)]))

        # x axis of female{i} (pointing to the head)
        x_axes = [(points[i] - points[i + 1]) / np.linalg.norm(points[i] - points[i + 1]) for i in range(n)]

        # z axis of female{i}: normal of the plane spanned by the neighboring links,
        # always upward which is guaranteed by the bend along the circle
        z_axes = []
        for i in range(n - 1):
            normal = np.cross(x_axes[i], x_axes[i + 1])
            normal /= np.linalg.norm(normal)
            if normal[2] < 0:
                normal = -normal
            z_axes.append(normal)
        # tail link: keep level as much as possible
        up = np.array([0, 0, 1.0])
        normal = up - up.dot(x_axes[-1]) * x_axes[-1]
        z_axes.append(normal / np.linalg.norm(normal))

        joint_yaw = [math.atan2(np.cross(x_axes[i], x_axes[i + 1]).dot(z_axes[i]), x_axes[i].dot(x_axes[i + 1])) \
                     for i in range(n - 1)]
        joint_roll = [0.0] + [math.atan2(np.cross(z_axes[i], z_axes[i + 1]).dot(x_axes[i + 1]), z_axes[i].dot(z_axes[i + 1])) \
                              for i in range(n - 1)]

        b = self.baselink_index
        rot = np.column_stack([x_axes[b], np.cross(z_axes[b], x_axes[b]), z_axes[b]])
        roll = math.atan2(rot[2, 1], rot[2, 2])
        pitch = -math.asin(np.clip(rot[2, 0], -1, 1))
        yaw = math.atan2(rot[1, 0], rot[0, 0])

        # approximate the CoG by the average of the link centers
        cog = np.mean([(points[i] + points[i + 1]) / 2 for i in range(n)], axis = 0)

        return joint_yaw, joint_roll, [roll, pitch, yaw], cog


# fly along the circle with the body on the sine curve on the cylinder like a dragon dance
class DragonDance(BaseState):
    def __init__(self, robot, shape, joint_yaw_names, joint_roll_names, prefix = 'dragon_dance', \
                 period = 50.0, loop = 1, amplitude = 0.1, ramp_time = 10.0, hold_time = 2.0):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'])

        self.shape = shape
        self.joint_names = joint_yaw_names + joint_roll_names
        self.period = rospy.get_param('~' + prefix + '/period', period) # period of one lap [sec]
        self.loop = rospy.get_param('~' + prefix + '/loop', loop)
        self.amplitude = rospy.get_param('~' + prefix + '/amplitude', amplitude) # amplitude of the sine curve [m]
        self.ramp_time = rospy.get_param('~' + prefix + '/ramp_time', ramp_time) # time to accelerate / decelerate [sec]
        self.hold_time = rospy.get_param('~' + prefix + '/hold_time', hold_time)
        self.nav_rate = rospy.get_param('~nav_rate', 20.0) # hz

    # CoG position and yaw in the world frame
    def toWorld(self, cog, yaw):
        c, s = math.cos(self.offset_yaw), math.sin(self.offset_yaw)
        pos = [c * cog[0] - s * cog[1] + self.offset_pos[0], \
               s * cog[0] + c * cog[1] + self.offset_pos[1], \
               cog[2] + self.offset_pos[2]]
        return pos, yaw + self.offset_yaw

    def execute(self, userdata):

        if self.robot.getFlightState() != self.robot.HOVER_STATE:
            rospy.logwarn(self.__class__.__name__ + ': the robot state ({}) is not at HOVER_STATE. preempted!'.format(self.robot.getFlightState()))
            return 'preempted'

        direction = self.shape.direction
        omega = 2 * math.pi / self.period
        accel = omega / self.ramp_time
        total_angle = 2 * math.pi * self.loop

        # place the cylinder frame so that the current robot is at theta = 0 without undulation
        # NOTE: the yaw of CoG frame is same as that of baselink (fc)
        _, _, base_rpy, cog = self.shape.calc(0, 0)
        self.offset_yaw = self.robot.getCogRPY()[2] - base_rpy[2]
        c, s = math.cos(self.offset_yaw), math.sin(self.offset_yaw)
        pos = self.robot.getCogPos()
        self.offset_pos = [pos[0] - (c * cog[0] - s * cog[1]), \
                           pos[1] - (s * cog[0] + c * cog[1]), \
                           pos[2] - cog[2]]
        rospy.loginfo(self.__class__.__name__ + ': the center position is [{:.3f}, {:.3f}]'.format(self.offset_pos[0], self.offset_pos[1]))

        dt = 1.0 / self.nav_rate
        r = rospy.Rate(self.nav_rate)
        progress = 0.0 # absolute angle traveled along the circle
        t = 0.0
        while progress < total_angle:

            if rospy.is_shutdown() or self.robot.getFlightState() != self.robot.HOVER_STATE:
                rospy.logerr(self.__class__.__name__ + ': the robot is not hovering, preempt!')
                return 'preempted'

            # trapezoidal angular velocity profile
            remain = total_angle - progress
            w = min(omega, accel * t, math.sqrt(2 * accel * remain))
            w = max(w, accel * dt) # avoid getting stuck at the start / end
            progress = min(progress + w * dt, total_angle)

            # the amplitude follows the speed
            theta = direction * progress
            amplitude = w / omega * self.amplitude
            joint_yaw, joint_roll, base_rpy, cog = self.shape.calc(theta, amplitude)
            self.robot.setJointAngle(self.joint_names, joint_yaw + joint_roll, timeout = -1)
            self.robot.rotateCog(base_rpy[0], base_rpy[1])

            # feedforward velocity by the numerical differentiation
            _, _, next_rpy, next_cog = self.shape.calc(theta + direction * w * dt, amplitude)
            target_pos, target_yaw = self.toWorld(cog, base_rpy[2])
            next_pos, next_yaw = self.toWorld(next_cog, next_rpy[2])
            target_vel = [(n - p) / dt for n, p in zip(next_pos, target_pos)]
            target_omega = math.atan2(math.sin(next_yaw - target_yaw), math.cos(next_yaw - target_yaw)) / dt
            self.robot.navigate(pos = target_pos, rot = [0, 0, target_yaw], \
                                lin_vel = target_vel, ang_vel = [0, 0, target_omega])

            t += dt
            r.sleep()

        # stop at the end point of the circle with the shape without undulation
        joint_yaw, joint_roll, base_rpy, cog = self.shape.calc(direction * total_angle, 0)
        target_pos, target_yaw = self.toWorld(cog, base_rpy[2])
        self.robot.navigate(pos = target_pos, rot = [0, 0, target_yaw], lin_vel = [0, 0, 0], ang_vel = [0, 0, 0])
        self.robot.setJointAngle(self.joint_names, joint_yaw + joint_roll, timeout = -1)
        self.robot.rotateCog(0, 0)
        rospy.loginfo(self.__class__.__name__ + ': finish {} loops'.format(self.loop))
        self.hold(self.hold_time, userdata.flags)

        return 'succeeded'


# Dragon dance demo for OrochiHx (full vectoring mode)
class OrochiHxDragonDanceDemo():
    def __init__(self):
        self.robot = OrochiHxInterface()
        rospy.sleep(1.0) # wait for joint updated

        # joint_roll{i} is only available in the full vectoring mode
        if self.robot.joint_roll_ctrl_pub.get_num_connections() == 0:
            rospy.logerr('dragon dance demo requires the full vectoring mode (joint_roll_ctrl is not subscribed)')
            return

        joint_state = self.robot.getJointState()
        joints = dict(zip(joint_state.name, joint_state.position))
        link_num = len([n for n in joints if n.startswith('joint_roll')])
        joint_yaw_names = ['joint_yaw{}'.format(i + 1) for i in range(link_num - 1)]
        joint_roll_names = ['joint_roll{}'.format(i + 1) for i in range(link_num)]
        joint_names = joint_yaw_names + joint_roll_names
        init_angles = [joints[n] for n in joint_names]
        rospy.loginfo('initial joint angles {}: {}'.format(joint_names, init_angles))

        # body along the circle without undulation
        radius = rospy.get_param('~dragon_dance/radius', 1.25)
        shape = DragonDanceShape(link_num = link_num, radius = radius, \
                                 wave_length = rospy.get_param('~dragon_dance/wave_length', 2 * math.pi * radius / 3), \
                                 direction = rospy.get_param('~dragon_dance/direction', -1)) # CW: same bending direction as the initial pose
        joint_yaw, joint_roll, _, _ = shape.calc(0, 0)
        arc_angles = joint_yaw + joint_roll

        self.sm_top = smach.StateMachine(outcomes=['succeeded', 'preempted'])
        self.sm_top.userdata.flags = {}
        self.sm_top.userdata.extra = {}

        with self.sm_top:
            smach.StateMachine.add('Start', Start(self.robot),
                                   transitions={'succeeded':'Arm',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            smach.StateMachine.add('Arm', Arm(self.robot),
                                   transitions={'succeeded':'Takeoff',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            smach.StateMachine.add('Takeoff', Takeoff(self.robot),
                                   transitions={'succeeded':'Move',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            # move to the start position of the dragon dance
            height = rospy.get_param('~dragon_dance/height', 1.5)
            smach.StateMachine.add('Move', WayPoint(self.robot, 'move', \
                                                    waypoints = [[radius, 0.0, height]], \
                                                    timeout = 30.0, hold_time = 2.0),
                                   transitions={'succeeded':'Straighten',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            smach.StateMachine.add('Straighten', Transform(self.robot, 'straighten', \
                                                           joint_names, [arc_angles], \
                                                           thresh = 0.05, timeout = 20.0, \
                                                           hold_time = 2.0),
                                   transitions={'succeeded':'DragonDance',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            smach.StateMachine.add('DragonDance', DragonDance(self.robot, shape, joint_yaw_names, joint_roll_names),
                                   transitions={'succeeded':'Restore',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            smach.StateMachine.add('Restore', Transform(self.robot, 'restore', \
                                                        joint_names, [init_angles], \
                                                        thresh = 0.05, timeout = 20.0, \
                                                        hold_time = 2.0),
                                   transitions={'succeeded':'Land',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            smach.StateMachine.add('Land', Land(self.robot),
                                   transitions={'succeeded':'succeeded',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            self.sis = smach_ros.IntrospectionServer('task_smach_server', self.sm_top, '/SM_ROOT')

        self.sis.start()
        outcome = self.sm_top.execute()

        self.sis.stop()


if __name__ == '__main__':

    rospy.init_node('orochi_hx_dragon_dance_demo')

    demo = OrochiHxDragonDanceDemo()
