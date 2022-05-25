import pyrealsense2 as real_sense
import numpy as np



class Camera():
    def __init__(self):
        self.pipeline = real_sense.pipeline()
        self.config = real_sense.config()

        pipeline_wrapper = real_sense.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        self.config.enable_stream(real_sense.stream.depth, 848, 480, real_sense.format.z16, 30)
        self.config.enable_stream(real_sense.stream.color, 640, 480, real_sense.format.bgr8, 30)

        profile = self.pipeline.start(self.config)

        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.align_stream = real_sense.align(real_sense.stream.color)

        for i in range(60):
            self.GetImage()


    def GetImage(self):
        while True:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align_stream.process(frames)

            aligned_depth_frame = aligned_frames.get_depth_frame()
            aligned_color_frame = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not aligned_color_frame:
                continue

            depth_frame = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(aligned_color_frame.get_data())

            return color_image,depth_frame,self.depth_scale


    def Stop(self):
        self.pipeline.stop()


