#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file MobileRobotShootingGame.py
 @brief Shooting game system with mobile robot oriented mixed reality
 @date $Date$


"""
import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

import numpy as np

# Import Service implementation class
# <rtc-template block="service_impl">

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
mobilerobotshootinggame_spec = ["implementation_id", "MobileRobotShootingGame",
                                "type_name",         "MobileRobotShootingGame",
                                "description",       "Shooting game system with mobile robot oriented mixed reality",
                                "version",           "1.0.0",
                                "vendor",            "ota",
                                "category",          "Game",
                                "activity_type",     "STATIC",
                                "max_instance",      "1",
                                "language",          "Python",
                                "lang_type",         "SCRIPT",
                                "conf.default.hit_thresh_distance", "1000",
                                "conf.default.hit_thresh_angle", "10",
                                "conf.default.hitpoint", "3",
                                "conf.default.shot_speed", "1.0",

                                "conf.__widget__.hit_thresh_distance", "slider.10",
                                "conf.__widget__.hit_thresh_angle", "slider.1",
                                "conf.__widget__.hitpoint", "text",
                                "conf.__widget__.shot_speed", "slider.0.01",
                                "conf.__constraints__.hit_thresh_angle", "hit_thresh_angle>0, hit_thresh_angle<90",

                                "conf.__type__.hit_thresh_distance", "int",
                                "conf.__type__.hit_thresh_angle", "int",
                                "conf.__type__.hitpoint", "int",
                                "conf.__type__.shot_speed", "float",

                                ""]
# </rtc-template>

##
# @class MobileRobotShootingGame
# @brief Shooting game system with mobile robot oriented mixed reality
#
#


class MobileRobotShootingGame(OpenRTM_aist.DataFlowComponentBase):

    ##
    # @brief constructor
    # @param manager Maneger Object
    #
    def __init__(self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

        joystick_button_arg = [None] * ((len(RTC._d_TimedBooleanSeq) - 4) / 2)
        self._d_joystick_button = RTC.TimedBooleanSeq(*joystick_button_arg)
        """
        """
        self._joystick_buttonIn = OpenRTM_aist.InPort(
            "joystick_button", self._d_joystick_button)
        player_pose_arg = [None] * ((len(RTC._d_TimedPose2D) - 4) / 2)
        self._d_player_pose = RTC.TimedPose2D(*player_pose_arg)
        """
        """
        self._player_poseIn = OpenRTM_aist.InPort(
            "player_pose", self._d_player_pose)
        enemy_pose_arg = [None] * ((len(RTC._d_TimedPose2D) - 4) / 2)
        self._d_enemy_pose = RTC.TimedPose2D(*enemy_pose_arg)
        """
        """
        self._enemy_poseIn = OpenRTM_aist.InPort(
            "enemy_pose", self._d_enemy_pose)
        joystick_button_enemy_arg = [None] * \
            ((len(RTC._d_TimedBooleanSeq) - 4) / 2)
        self._d_joystick_button_enemy = RTC.TimedBooleanSeq(
            *joystick_button_enemy_arg)
        """
        """
        self._enemy_buttonIn = OpenRTM_aist.InPort(
            "enemy_button", self._d_joystick_button_enemy)
        game_state_arg = [None] * ((len(RTC._d_TimedStringSeq) - 4) / 2)
        self._d_game_state = RTC.TimedStringSeq(RTC.Time(0, 0), [])
        """
        """
        self._game_stateOut = OpenRTM_aist.OutPort(
            "game_state", self._d_game_state)

        # initialize of configuration-data.
        # <rtc-template block="init_conf_param">
        """

         - Name:  hit_thresh_distance
         - DefaultValue: 1000
        """
        self._hit_thresh_distance = [1000]
        """

         - Name:  hit_thresh_angle
         - DefaultValue: 10
        """
        self._hit_thresh_angle = [10]
        """

         - Name:  hitpoint
         - DefaultValue: 3
        """
        self._hitpoint = [3]
        """

         - Name:  shot_speed
         - DefaultValue: 1.0
        """
        self._shot_speed = [1.0]

        # </rtc-template>

        self.hit_count = 0
        self.damage_count = 0

    ##
    #
    # The initialize action (on CREATED->ALIVE transition)
    # formaer rtc_init_entry()
    #
    # @return RTC::ReturnCode_t
    #
    #
    def onInitialize(self):
        # Bind variables and configuration variable
        self.bindParameter("hit_thresh_distance",
                           self._hit_thresh_distance, "1000")
        self.bindParameter("hit_thresh_angle", self._hit_thresh_angle, "10")
        self.bindParameter("hitpoint", self._hitpoint, "3")
        self.bindParameter("shot_speed", self._shot_speed, "1.0")

        # Set InPort buffers
        self.addInPort("joystick_button", self._joystick_buttonIn)
        self.addInPort("player_pose", self._player_poseIn)
        self.addInPort("enemy_pose", self._enemy_poseIn)
        self.addInPort("enemy_button", self._enemy_buttonIn)

        # Set OutPort buffers
        self.addOutPort("game_state", self._game_stateOut)

        # Set service provider to Ports

        # Set service consumers to Ports

        # Set CORBA Service Ports

        return RTC.RTC_OK

    #    ##
    #    #
    #    # The finalize action (on ALIVE->END transition)
    #    # formaer rtc_exiting_entry()
    #    #
    #    # @return RTC::ReturnCode_t
    #
    #    #
    # def onFinalize(self):
    #
    #    return RTC.RTC_OK

    #    ##
    #    #
    #    # The startup action when ExecutionContext startup
    #    # former rtc_starting_entry()
    #    #
    #    # @param ec_id target ExecutionContext Id
    #    #
    #    # @return RTC::ReturnCode_t
    #    #
    #    #
    # def onStartup(self, ec_id):
    #
    #    return RTC.RTC_OK

    #    ##
    #    #
    #    # The shutdown action when ExecutionContext stop
    #    # former rtc_stopping_entry()
    #    #
    #    # @param ec_id target ExecutionContext Id
    #    #
    #    # @return RTC::ReturnCode_t
    #    #
    #    #
    # def onShutdown(self, ec_id):
    #
    #    return RTC.RTC_OK

        ##
        #
        # The activated action (Active state entry action)
        # former rtc_active_entry()
        #
        # @param ec_id target ExecutionContext Id
        #
        # @return RTC::ReturnCode_t
        #
        #
    def onActivated(self, ec_id):

        self._d_game_state.data = ["win or loose",
                                   "player's action", "player's status",
                                   "enemy's action", "enemy's status"]
        print self._d_game_state.data

        self._d_player_pose.data = RTC.Pose2D(
            RTC.Point2D(0.0, 0.0), 3.14 / 4)
        self._d_enemy_pose.data = RTC.Pose2D(
            RTC.Point2D(100.0, 100.0), 0.0)
        print "player pose: ", self._d_player_pose.data
        print "enemy pose:  ", self._d_enemy_pose.data

        self.hit_count = 0

        return RTC.RTC_OK

        ##
        #
        # The deactivated action (Active state exit action)
        # former rtc_active_exit()
        #
        # @param ec_id target ExecutionContext Id
        #
        # @return RTC::ReturnCode_t
        #
        #
    def onDeactivated(self, ec_id):

        return RTC.RTC_OK

        ##
        #
        # The execution action that is invoked periodically
        # former rtc_active_do()
        #
        # @param ec_id target ExecutionContext Id
        #
        # @return RTC::ReturnCode_t
        #
        #

    # def enemyInRange(self, player_pose, enemy_pose, hit_thresh_distance, hit_thresh_angle):
    #
    #     slope = np.tan(player_pose.heading)
    #     slope_thresh = float(hit_thresh_angle) / 180 * np.pi
    #     intercept = player_pose.position.y - slope * player_pose.position.x
    #     print "slope  ", slope
    #     print "slopth ", slope_thresh
    #     print "inter  ", intercept
    #
    #     if enemy_pose.position.y > (slope - slope_thresh) * enemy_pose.position.x + intercept:
    #         print "above"
    #     if enemy_pose.position.y < (slope + slope_thresh) * enemy_pose.position.x + intercept:
    #         print "below"
    #
    #     if enemy_pose.position.y > (slope - slope_thresh) * enemy_pose.position.x + intercept and \
    #        enemy_pose.position.y < (slope + slope_thresh) * enemy_pose.position.x + intercept:
    #         distance = np.sqrt((player_pose.position.x - enemy_pose.position.x)**2
    #                            + (player_pose.position.y - enemy_pose.position.y)**2)
    #         print "player-enemy distance: ", distance
    #         if distance < hit_thresh_distance:
    #             return True
    def enemyInRange(self, player_pose, enemy_pose, hit_thresh_distance, hit_thresh_angle):

        def angle(x, y):
            dot_xy = np.dot(x, y)
            norm_x = np.linalg.norm(x)
            norm_y = np.linalg.norm(y)
            cos = dot_xy / (norm_x * norm_y)
            rad = np.arccos(cos)
            theta = rad * 180 / np.pi
            return theta

        vec_heading = np.array(
            [np.cos(player_pose.heading), np.sin(player_pose.heading)])
        vec_player_enemy = np.array(
            [enemy_pose.position.x - player_pose.position.x,  enemy_pose.position.y - player_pose.position.y])
        print "vec_heading:      ", vec_heading
        print "vec_player_enemy: ", vec_player_enemy

        angle = angle(vec_heading, vec_player_enemy)
        distance = np.linalg.norm(vec_player_enemy)

        print "angle:    ", angle
        print "distance: ", distance
        if abs(angle) > hit_thresh_angle:
            print "out of range"
        if distance > hit_thresh_distance:
            print "too far"
        if abs(angle) < hit_thresh_angle and distance < hit_thresh_distance:
            return True

    def onExecute(self, ec_id):
        # if self.hit_count <= self._hitpoint[0]:
        #     self._d_game_state.data = ["win or loose",
        #                                "event", "player action", "enemy action"]
        if self._player_poseIn.isNew():
            self._d_player_pose = self._player_poseIn.read()
            # print self._d_player_pose.data

        if self._enemy_poseIn.isNew():
            self._d_enemy_pose = self._enemy_poseIn.read()
            # print self._d_enemy_pose.data

        if self._joystick_buttonIn.isNew():
            self._d_joystick_button = self._joystick_buttonIn.read()
            # print self._d_joystick_button.data
            if self._d_joystick_button.data[0] == 1:
                print "player shoots"
                self._d_game_state.data[2] = "player shoots"

                if self.enemyInRange(self._d_player_pose.data, self._d_enemy_pose.data,
                                     self._hit_thresh_distance[0], self._hit_thresh_angle[0]):

                    self.hit_count = self.hit_count + 1
                    print self.hit_count, " hit on enemy"
                    self._d_game_state.data[4] = "hit"

                    if self.hit_count >= self._hitpoint[0]:
                        print "player win"
                        self._d_game_state.data[0] = "win"
                        self._d_game_state.data.append("player win")

        if self._enemy_buttonIn.isNew():
            self._d_enemy_button = self._enemy_buttonIn.read()
            # print self._d_enemy_button.data
            if self._d_enemy_button.data[0] == 1:
                print "enemy shoots"
                self._d_game_state.data[3] = "enemy shoots"

                if self.enemyInRange(self._d_enemy_pose.data, self._d_player_pose.data,
                                     self._hit_thresh_distance[0], self._hit_thresh_angle[0]):

                    self.damage_count = self.damage_count + 1
                    print self.damage_count, "hit on player"
                    self._d_game_state.data[2] = "hit"

                    if self.damage_count >= self._hitpoint[0]:
                        print "player loose"
                        self._d_game_state.data[0] = "loose"
                        self._d_game_state.data.append("player loose")

        # print self.hit_count
        # print self._d_game_state.data
        self._game_stateOut.write()
        return RTC.RTC_OK

    #    ##
    #    #
    #    # The aborting action when main logic error occurred.
    #    # former rtc_aborting_entry()
    #    #
    #    # @param ec_id target ExecutionContext Id
    #    #
    #    # @return RTC::ReturnCode_t
    #    #
    #    #
    # def onAborting(self, ec_id):
    #
    #    return RTC.RTC_OK

    #    ##
    #    #
    #    # The error action in ERROR state
    #    # former rtc_error_do()
    #    #
    #    # @param ec_id target ExecutionContext Id
    #    #
    #    # @return RTC::ReturnCode_t
    #    #
    #    #
    # def onError(self, ec_id):
    #
    #    return RTC.RTC_OK

    #    ##
    #    #
    #    # The reset action that is invoked resetting
    #    # This is same but different the former rtc_init_entry()
    #    #
    #    # @param ec_id target ExecutionContext Id
    #    #
    #    # @return RTC::ReturnCode_t
    #    #
    #    #
    # def onReset(self, ec_id):
    #
    #    return RTC.RTC_OK

    #    ##
    #    #
    #    # The state update action that is invoked after onExecute() action
    #    # no corresponding operation exists in OpenRTm-aist-0.2.0
    #    #
    #    # @param ec_id target ExecutionContext Id
    #    #
    #    # @return RTC::ReturnCode_t
    #    #

    #    #
    # def onStateUpdate(self, ec_id):
    #
    #    return RTC.RTC_OK

    #    ##
    #    #
    #    # The action that is invoked when execution context's rate is changed
    #    # no corresponding operation exists in OpenRTm-aist-0.2.0
    #    #
    #    # @param ec_id target ExecutionContext Id
    #    #
    #    # @return RTC::ReturnCode_t
    #    #
    #    #
    # def onRateChanged(self, ec_id):
    #
    #    return RTC.RTC_OK


def MobileRobotShootingGameInit(manager):
    profile = OpenRTM_aist.Properties(
        defaults_str=mobilerobotshootinggame_spec)
    manager.registerFactory(profile,
                            MobileRobotShootingGame,
                            OpenRTM_aist.Delete)


def MyModuleInit(manager):
    MobileRobotShootingGameInit(manager)

    # Create a component
    comp = manager.createComponent("MobileRobotShootingGame")


def main():
    mgr = OpenRTM_aist.Manager.init(sys.argv)
    mgr.setModuleInitProc(MyModuleInit)
    mgr.activateManager()
    mgr.runManager()


if __name__ == "__main__":
    main()
