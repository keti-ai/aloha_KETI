import asyncio
import datetime
import logging
from typing import Any, Dict

import cv2
from aiortc import VideoStreamTrack
from av import VideoFrame

from ketirtc_robot.camera.cam_base import Camera
import numpy as np

logger = logging.getLogger(__name__)

class MultiCam(Camera):
    def __init__(self, camera_index_list):
        Camera.__init__(self)
        self._camera_index = camera_index_list

    def get_video_stream_track(self) -> VideoStreamTrack:
        return CV2VideoStreamTrack(self._camera_index)


class CV2VideoStreamTrack(VideoStreamTrack):
    __started_devices: Dict[str, Any] = {}

    @classmethod
    def _release_camera(cls, cam_id):
        cls.__started_devices.pop(cam_id)

    def __new__(cls, device_index=None, *args, **kwargs):
        if device_index is None:
            device_index = 0

        if device_index[0] not in cls.__started_devices:
            logger.debug(f'new camera instance: {device_index}')
            sensor = super(CV2VideoStreamTrack, cls).__new__(cls)
            cls.__started_devices[device_index[0]] = sensor
            sensor._init_cam(device_index)
        else:
            sensor = cls.__started_devices[device_index[0]]
        return sensor

    def __init__(self, device_index: str = None):
        super(CV2VideoStreamTrack, self).__init__()
        self._instance_count += 1
        self.last_process_time = datetime.datetime.now()


    def _init_cam(self, device_index_list):         # [FRONT, TOP, LEFT, RIGHT] ===> [4, 0 ,2 ,6]
        self.cam_list = []
        self.width = 640
        self.height = 360
        self.fps = 30
        self.last_image = None


        for i in device_index_list:
            if i == device_index_list[0]:                                       # for front cam resolution upgrade.
                self._instance_count = 0
                self.kind = "video"
                _video = cv2.VideoCapture(i)  # name

                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                _video.set(cv2.CAP_PROP_FOURCC, fourcc)

                _video.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                _video.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                _video.set(cv2.CAP_PROP_FPS, 30)
                print('camid:{}, width:{}, height:{}, fps:{}'.format(i, _video.get(cv2.CAP_PROP_FRAME_WIDTH),
                                                                            _video.get(cv2.CAP_PROP_FRAME_HEIGHT),
                                                                            _video.get(cv2.CAP_PROP_FPS)))

                logger.debug('camid:{}, width:{}, height:{}, fps:{}'.format(i, _video.get(cv2.CAP_PROP_FRAME_WIDTH),
                                                             _video.get(cv2.CAP_PROP_FRAME_HEIGHT),
                                                             _video.get(cv2.CAP_PROP_FPS)))
                self.cam_list.append(_video)

            elif i == device_index_list[1]:
                self.kind = "video"
                _video = cv2.VideoCapture(i)  # name

                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                _video.set(cv2.CAP_PROP_FOURCC, fourcc)
                _video.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                _video.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                _video.set(cv2.CAP_PROP_FPS, 30)
                print('camid:{}, width:{}, height:{}, fps:{}'.format(i, _video.get(cv2.CAP_PROP_FRAME_WIDTH),
                                                                     _video.get(cv2.CAP_PROP_FRAME_HEIGHT),
                                                                     _video.get(cv2.CAP_PROP_FPS)))

                logger.debug('camid:{}, width:{}, height:{}'.format(i, _video.get(cv2.CAP_PROP_FRAME_WIDTH),
                                                                    _video.get(cv2.CAP_PROP_FRAME_HEIGHT)))
                self.cam_list.append(_video)

            elif i == device_index_list[2]:
                self.kind = "video"
                _video = cv2.VideoCapture(i)  # name

                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                _video.set(cv2.CAP_PROP_FOURCC, fourcc)
                _video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                _video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                _video.set(cv2.CAP_PROP_FPS, 30)
                print('camid:{}, width:{}, height:{}, fps:{}'.format(i, _video.get(cv2.CAP_PROP_FRAME_WIDTH),
                                                                     _video.get(cv2.CAP_PROP_FRAME_HEIGHT),
                                                                     _video.get(cv2.CAP_PROP_FPS)))

                logger.debug('camid:{}, width:{}, height:{}'.format(i, _video.get(cv2.CAP_PROP_FRAME_WIDTH),
                                                             _video.get(cv2.CAP_PROP_FRAME_HEIGHT)))
                self.cam_list.append(_video)

            elif i == device_index_list[3]:
                self.kind = "video"
                _video = cv2.VideoCapture(i)  # name

                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                _video.set(cv2.CAP_PROP_FOURCC, fourcc)
                _video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                _video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                _video.set(cv2.CAP_PROP_FPS, 30)
                print('camid:{}, width:{}, height:{}, fps:{}'.format(i, _video.get(cv2.CAP_PROP_FRAME_WIDTH),
                                                                     _video.get(cv2.CAP_PROP_FRAME_HEIGHT),
                                                                     _video.get(cv2.CAP_PROP_FPS)))

                logger.debug('camid:{}, width:{}, height:{}'.format(i, _video.get(cv2.CAP_PROP_FRAME_WIDTH),
                                                             _video.get(cv2.CAP_PROP_FRAME_HEIGHT)))
                self.cam_list.append(_video)

        print(self.cam_list)

    async def camera_capture(self, cam):
        ret, img = cam.read()
        return img

    async def all_cameras_capture_async(self):
        img_list = []
        for camera in self.cam_list:
            img = self.camera_capture(camera)
            img_list.append(img)

        return img_list

        return await asyncio.gather(*tasks)

    def all_cameras_capture_sync(self):
        img_list = []
        for camera in self.cam_list:
            ret, img = camera.read()
            img_list.append(img)

        return img_list

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        multi_img = np.zeros((1300, 2100, 3), np.uint8)  # 화면은 가로 3개 세로 2개 배치
        img_list = self.all_cameras_capture_sync()

        multi_img[10:1290, 10:730] = cv2.rotate(img_list[0], cv2.ROTATE_90_CLOCKWISE)            # FRONT
        multi_img[10:730, 780:2060] = img_list[1]                                                       # TOP
        multi_img[790:1270, 760:1400] = img_list[2]                                                     # LEFT
        multi_img[790:1270, 1430:2070] = img_list[3]                                                    # RIGHT
        getimagetime = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        cv2.putText(multi_img, getimagetime, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
        # print("{}".format(datetime.datetime.now() - self.last_process_time))
        # self.last_process_time = datetime.datetime.now()

        # self.last_image = multi_img

        frame = VideoFrame.from_ndarray(multi_img, format='bgr24')
        frame.pts = pts
        frame.time_base = time_base

        return frame


    def stop(self):
        self._instance_count -= 1
        logger.debug(f'number of camera instances: {self._instance_count}')
        if self._instance_count <= 0:
            print(f'stop camera {self.device_id}')
            CV2VideoStreamTrack._release_camera(self.device_id)
            self._video.release()
            super().stop()