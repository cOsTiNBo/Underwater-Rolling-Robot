import rclpy
from rclpy.node import Node
import cv2
import os
from datetime import datetime

#Create a new node for the camera
class CameraLoggerNode(Node):
    def __init__(self):
        super().__init__('camera_logger')

        #Video recording path
        self.folder = os.path.expanduser("~/videos")
        os.makedirs(self.folder, exist_ok=True)

        #Open the camera port
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open USB camera (/dev/video0).")
            return

        #Set the image size and framerate
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(self.cap.get(cv2.CAP_PROP_FPS)) or 20

        now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        #Create a new video everytime the node starts
        self.filename = os.path.join(self.folder, f"video_{now}.avi")

        # Video codec converter
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(self.filename, fourcc, fps, (width, height))

        self.timer = self.create_timer(1.0 / fps, self.capture_frame)
        self.get_logger().info(f"Recording video to {self.filename}")

    #Start recording
    def capture_frame(self):
        ret, frame = self.cap.read()
        if ret:
            #Add a watermark over the video with the date and time
            self.out.write(frame)
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(
                frame,
                timestamp,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 255),
                2,
                cv2.LINE_AA
            )
            self.out.write(frame)

    #Stopping the recording and closing the node
    def destroy_node(self):
        self.get_logger().info("Stopping video recording")
        if self.cap and self.cap.isOpened():
            self.cap.release()
        if self.out:
            self.out.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

