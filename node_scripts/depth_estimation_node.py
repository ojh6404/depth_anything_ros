#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import pycuda.autoinit
import pycuda.driver as cuda
import tensorrt as trt

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from utils import preprocess, INPUT_WIDTH, INPUT_HEIGHT


class DepthAnythingNode(object):
    def __init__(self):
        super(DepthAnythingNode, self).__init__()
        self.bridge = CvBridge()
        self.engine = rospy.get_param("~model_path", None)
        self.depth_scale = rospy.get_param("~depth_scale", 1.0)
        if self.engine is None:
            rospy.logerr("Model path not provided")
            return
        self.image_msg = None
        self.sub_image = rospy.Subscriber("~input_image", Image, self.image_cb, queue_size=1)
        self.pub_depth = rospy.Publisher("~depth_registered/image_rect", Image, queue_size=1)

        # Create logger and load the TensorRT engine
        logger = trt.Logger(trt.Logger.WARNING)
        with open(self.engine, "rb") as f, trt.Runtime(logger) as runtime:
            engine = runtime.deserialize_cuda_engine(f.read())

        while not rospy.is_shutdown():
            if self.image_msg is not None:
                image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="rgb8")
                orig_h, orig_w = image.shape[:2]
                image = preprocess(image)

                with engine.create_execution_context() as context:
                    input_shape = context.get_tensor_shape("input")
                    output_shape = context.get_tensor_shape("output")
                    h_input = cuda.pagelocked_empty(trt.volume(input_shape), dtype=np.float32)
                    h_output = cuda.pagelocked_empty(trt.volume(output_shape), dtype=np.float32)
                    d_input = cuda.mem_alloc(h_input.nbytes)
                    d_output = cuda.mem_alloc(h_output.nbytes)
                    stream = cuda.Stream()

                    # Copy the input image to the pagelocked memory
                    np.copyto(h_input, image.ravel())

                    # Copy the input to the GPU, execute the inference, and copy the output back to the CPU
                    cuda.memcpy_htod_async(d_input, h_input, stream)
                    context.execute_async_v2(bindings=[int(d_input), int(d_output)], stream_handle=stream.handle)
                    cuda.memcpy_dtoh_async(h_output, d_output, stream)
                    stream.synchronize()
                    depth = h_output

                # Process the depth output
                depth = np.reshape(depth, (INPUT_WIDTH, INPUT_HEIGHT)) * self.depth_scale
                depth = cv2.resize(depth, (orig_w, orig_h), interpolation=cv2.INTER_CUBIC)
                depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
                depth_msg.header.stamp = rospy.Time.now()
                depth_msg.header.frame_id = self.image_msg.header.frame_id
                self.pub_depth.publish(depth_msg)
                self.image_msg = None

    def image_cb(self, msg):
        self.image_msg = msg


if __name__ == "__main__":
    rospy.init_node("depth_anything_node")
    node = DepthAnythingNode()
    rospy.spin()
