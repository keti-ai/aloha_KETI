


import rospy
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from ketirtc_robot.camera.cam_base import Camera
from aiortc import VideoStreamTrack

from av import VideoFrame

class ROSCV2StreamCamera(Camera):
    def __init__(self, subscrib_topic):
        super(ROSCV2StreamCamera, self).__init__()  
        self.subscrib_topic = subscrib_topic     
        # self.image_pub = rospy.Publisher("camera_bridge_test",Image)

        self.bridge = CvBridge()
        
        rospy.loginfo(f"camera topic: {self.subscrib_topic}")

        self.image_sub = rospy.Subscriber(self.subscrib_topic, Image, self.callback)
        self.last_image = None
        self.tracks = set()

    def callback(self,data):
        try:

            # rospy.loginfo("new camera frame event")
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.last_image = cv_image

            # rospy.loginfo("DEBUG: camera frame")
            # cv2.imshow("Image window", cv_image)
            # cv2.waitKey(1)
        except CvBridgeError as e:          
            rospy.loginfo("DEBUG: camera frame")
        # finally:
        #     rospy.loginfo("DEBUG: callback finally")
        # rospy.loginfo("Callback end")


    def get_image(self):        
        return self.last_image
    
    def get_video_stream_track(self) -> VideoStreamTrack:
            track = ROSCV2StreamTrack(self)
            return track

class ROSCV2StreamTrack(VideoStreamTrack):
    def __init__(self, camera) -> None:
        self.camera = camera
    
    async def recv(self):
        rospy.loginfo("ROS streamtrack recv begin")
        pts, time_base = await self.next_timestamp()
        # TODO: check timestamp and fps ...
        image = self.camera.get_image()

        frame = VideoFrame.from_ndarray(image, format='bgr8')
        frame.pts = pts
        frame.time_base = time_base
        rospy.loginfo("ROS streamtrack recv return")
        return frame
