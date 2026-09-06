#!/usr/bin/env python3

import time
import depthai as dai
import rclpy

from rclpy.node import Node
from std_msgs.msg import Empty, Int32, Float32MultiArray


MODEL = "/home/team1/cone_yolo11n_rvc2.tar.xz"
FPS = 10.0
CONF = 0.25


class Detector(Node):

    def __init__(self):
        super().__init__("manta_direct_cone_detector")

        self.count_pub = self.create_publisher(
            Int32,
            "/cone/count_0",
            10
        )

        self.det_pub = self.create_publisher(
            Float32MultiArray,
            "/cone/detections_0",
            10
        )

        self.hb_pub = self.create_publisher(
            Empty,
            "/manta/vision/heartbeat",
            10
        )

        self.hazard_pub = self.create_publisher(
            Int32,
            "/manta/vision/hazard_state",
            10
        )

    def run(self):

        with dai.Pipeline() as pipeline:

            cam = (
                pipeline
                .create(dai.node.Camera)
                .build(dai.CameraBoardSocket.CAM_A)
            )

            camera_output = cam.requestOutput(
                (416, 416),
                dai.ImgFrame.Type.BGR888p,
                dai.ImgResizeMode.CROP,
                FPS
            )

            archive = dai.NNArchive(MODEL)

            detector = pipeline.create(
                dai.node.DetectionNetwork
            )

            detector.setNNArchive(archive)
            detector.setConfidenceThreshold(CONF)
            detector.input.setBlocking(False)

            camera_output.link(
                detector.input
            )

            frame_q = (
                detector
                .passthrough
                .createOutputQueue()
            )

            det_q = (
                detector
                .out
                .createOutputQueue()
            )

            self.get_logger().info(
                "Starting DIRECT OAK-D YOLO pipeline"
            )

            pipeline.start()

            self.get_logger().info(
                "DIRECT OAK-D YOLO RUNNING"
            )

            while rclpy.ok() and pipeline.isRunning():

                # These two packets correspond to one inference cycle.
                frame = frame_q.get()
                packet = det_q.get()

                if frame is None or packet is None:
                    continue

                detections = list(packet.detections)

                count = len(detections)

                self.count_pub.publish(
                    Int32(data=count)
                )

                values = []

                for det in detections:

                    values.extend([
                        float(det.xmin),
                        float(det.ymin),
                        float(det.xmax),
                        float(det.ymax),
                        float(det.confidence),
                    ])

                msg = Float32MultiArray()
                msg.data = values

                self.det_pub.publish(msg)

                # Existing MANTA semantics:
                # 0 = clear
                # 1 = one cone
                # 2 = two-or-more cones
                if count >= 2:
                    hazard = 2
                elif count == 1:
                    hazard = 1
                else:
                    hazard = 0

                self.hazard_pub.publish(
                    Int32(data=hazard)
                )

                self.hb_pub.publish(
                    Empty()
                )


def main():

    rclpy.init()

    node = Detector()

    try:
        node.run()

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        node.get_logger().error(
            f"DIRECT CAMERA FAILURE: {repr(exc)}"
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
