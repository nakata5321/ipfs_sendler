#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import ipfshttpclient
from pinatapy import PinataPy
from ipfs_sendler.url_generator import create_url
from ipfs_sendler.url_generator import update_url


def _pin_to_pinata(filename: str, config: dict) -> None:
    """
    :param filename: full name of a recorded video
    :type filename: str
    :param config: dictionary containing all the configurations
    :type config: dict
    pinning files in pinata to make them broadcasted around ipfs
    """
    pinata_api = config["pinata"]["pinata_api"]  # pinata credentials
    pinata_secret_api = config["pinata"]["pinata_secret_api"]
    if pinata_api and pinata_secret_api:
        pinata = PinataPy(pinata_api, pinata_secret_api)
        pinata.pin_file_to_ipfs(
            filename
        )  # here we actually send the entire file to pinata, not just its hash. It will
        # remain the same as if published locally, cause the content is the same.
        rospy.logwarn("File sent")


def send(filename: str, config: dict) -> str:
    """
    :param filename: full name of a recorded video
    :type filename: str
    :param config: dictionary containing all the configurations
    :type config: dict
    :return: shorted url
    :rtype: str

    """
    try:
        client = ipfshttpclient.connect()  # establish connection to local ipfs node
        res = client.add(filename)  # publish video locally
        hash = res["Hash"]  # get its hash of form Qm....
        rospy.loginfo("Published to IPFS")
        rospy.logdebug("Published to IPFS, hash: " + hash)
        keyword, link = create_url(config)
        update_url(
            keyword, hash, config
        )  # after publishing file in ipfs locally, which is pretty fast, update
        # the link on the qr code so that it redirects now to the gateway with a published file.
    except Exception as e:
        rospy.logerr("Error while publishing to IPFS. Error: ", e)
    try:
        # It may take some for the gateway node to find the file, so we need to pin it in pinata
        # pinning to pinata
        rospy.loginfo("Camera is sending file to pinata")
        _pin_to_pinata(filename, config)
        rospy.logdebug("Updating URL")
    except Exception as e:
        rospy.logerr("Error while pinning to pinata. Error: ", e)
    return link
