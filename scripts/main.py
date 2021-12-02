#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml
import rospy
from rospkg import RosPack
from std_msgs.msg import String
from ipfs_sendler.ipfs_send import send
from ipfs_sendler.qr_generator import create_qr


def ipfs_send_callback(ros_data: String) -> None:
    if ros_data.data == "stop":  # check that video recording is over
        rospy.sleep(5)
        link = send(video_path, config)  # send video to ipfs
        create_qr(dir_path, link, config)


if __name__ == "__main__":
    rospy.init_node("scripts", anonymous=True, disable_signals=True)
    rospy.loginfo("node initializing")
    dir_path = rospy.get_param("~dir_path")
    config = yaml.load(rospy.get_param("~config"), Loader=yaml.FullLoader)
    rospy.loginfo("node is up")
    # find path to the video
    rospack = RosPack()
    video_saver_path = rospack.get_path("video_saver")
    video_path = video_saver_path + "/videos/" + "output.mp4"

    video_ending_sub = rospy.Subscriber("/film", String, ipfs_send_callback)
    rospy.spin()
