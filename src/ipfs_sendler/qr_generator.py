import rospy
import qrcode
from rospkg import RosPack
from PIL import Image
from shutil import copyfile


def create_qr(dirname: str, link: str, config: dict = None) -> str:
    """
    :param dirname: path to the project ending with .../ipfs_sendler
    :type dirname: str
    :param link: full yourls url. E.g. url.today/6b
    :type link: str
    :param config: dictionary containing all the configurations
    :type config: dict
    :return: full filename of a resulted qr-code
    :rtype: str

    This is a qr-creating submodule. Inserts a Multi-Agent.io logo inside the qr and adds logos aside if required
    """

    rospy.loginfo("start creating QR")
    rospy.logdebug("add 'Multi-Agent.io logo' to QR")
    inpic_s = 150  # size of Multi-Agent.io logo in pixels
    multi_agent = Image.open(dirname + "/misc/Multi_Agent.jpg").resize(
        (inpic_s, inpic_s)  # resize logo if it's not the demanded size
    )
    rospy.logdebug(f"create QR with link - {link} ")
    qr_big = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_H)
    qr_big.add_data("https://" + link)
    qr_big.make()
    img_qr_big = qr_big.make_image().convert(
        "RGB"
    )  # some standard code to create qr-code with a python lib

    pos = (
        (img_qr_big.size[0] - multi_agent.size[0]) // 2,
        (img_qr_big.size[1] - multi_agent.size[1]) // 2,
    )  # position to insert to logo right in the center of a qr-code

    img_qr_big.paste(multi_agent, pos)  # insert logo

    qrpic = dirname + "/misc/" + "qr.png"
    rospy.logdebug(f"save QR. Find here {qrpic}")
    img_qr_big.save(qrpic)  # saving picture for further printing with a timestamp

    # save image to gaka-chu frontend if needed

    if config["general"]["gaka-chu_frontend"] == "enable":
        rospy.logdebug("save image to gaka-chu frontend")
        rospack = RosPack()
        dirname = rospack.get_path("gakachu-backend")
        picture_path = dirname + "/dist/assets/static/result.png"
        copyfile(qrpic, picture_path)

    rospy.loginfo("qr generated")
    return qrpic
