import asyncio
import datetime
import fractions
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

AUDIO_PTIME = 0.020  # 20ms audio packetization
VIDEO_CLOCK_RATE = 90000
VIDEO_PTIME = 1 / 60  # 30fps
VIDEO_TIME_BASE = fractions.Fraction(1, VIDEO_CLOCK_RATE)

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
        self.frame = None
        self.camera_setup = asyncio.Event()

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
        self.task = asyncio.create_task(self.get_frame())
        print('camera init')

    async def recv(self):
        if self.frame is None:
            await self.camera_setup.wait()
            # self.camera_setup.clear()
        pts, time_base = await self.next_timestamp()
        self.frame.pts = pts
        self.frame.time_base = time_base
        return self.frame


    async def capture_frames(self):
        try:
            loop = asyncio.get_event_loop()
            tasks = [loop.run_in_executor(None, self.read_frame, cam) for cam in self.cam_list]
            frame = await asyncio.gather(*tasks)
            return frame
        except Exception as e:
            print("ERROR : {}".format(e))

    def fake_time_stamp(self):
        self.frame.pts += int(VIDEO_PTIME * VIDEO_CLOCK_RATE)
        self.frame.time_base = VIDEO_TIME_BASE

    def read_frame(self, cam):
        ret, frame = cam.read()
        if not ret:
            raise RuntimeError("Failed to read frame from camera")
        # 프레임 처리 코드...
        return frame

    async def get_frame(self):
        while True:
            img_list = await self.capture_frames()
            multi_img = np.zeros((1300, 2100, 3), np.uint8)  # 화면은 가로 3개 세로 2개 배치

            multi_img[10:1290, 10:730] = cv2.rotate(img_list[0], cv2.ROTATE_90_CLOCKWISE)            # FRONT
            multi_img[10:730, 780:2060] = img_list[1]                                                       # TOP
            multi_img[790:1270, 760:1400] = img_list[2]                                                     # LEFT
            multi_img[790:1270, 1430:2070] = img_list[3]                                                    # RIGHT
            getimagetime = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            cv2.putText(multi_img, getimagetime, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
            self.frame = VideoFrame.from_ndarray(multi_img, format='bgr24')
            # self.frame.pts = pts
            # self.frame.time_base = time_base

            if self.last_image is None:
                self.camera_setup.set()

            self.last_image = multi_img
            await asyncio.sleep(0)


    def stop(self):
        self._instance_count -= 1
        logger.debug(f'number of camera instances: {self._instance_count}')
        if self._instance_count <= 0:
            print(f'stop camera {self.device_id}')
            CV2VideoStreamTrack._release_camera(self.device_id)
            self._video.release()
            for cam in self.cam_list:
                cam.release()

            super().stop()