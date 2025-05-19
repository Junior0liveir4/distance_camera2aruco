
from is_wire.core import Channel, Subscription, Message
from is_msgs.image_pb2 import Image
from scipy.spatial.transform import Rotation as R
import math
import numpy as np
import time
import cv2
import os

def to_np(input_image):
    if isinstance(input_image, np.ndarray):
        output_image = input_image
    elif isinstance(input_image, Image):
        buffer = np.frombuffer(input_image.data, dtype=np.uint8)
        output_image = cv2.imdecode(buffer, flags=cv2.IMREAD_COLOR)
    else:
        output_image = np.array([], dtype=np.uint8)
    return output_image

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

def save_frame_image(frame, index, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    filename = os.path.join(output_dir, f"frame_{index}.jpg")
    cv2.imwrite(filename, frame)
    print(f"[INFO] Frame salvo como {filename}")

def record_video_loop(output_path, frame_size, fps, channel, subscription, undistort_params):
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    video_writer = cv2.VideoWriter(output_path, fourcc, fps, frame_size)
    print(f"[INFO] Iniciando gravação em '{output_path}' (pressione 'r' para parar)")

    while True:
        msg = channel.consume()
        if type(msg) != bool:
            img = msg.unpack(Image)
            frame = to_np(img)

        K1, dist1, nK1, roi1 = undistort_params
        dst = cv2.undistort(frame, K1, dist1, None, nK1)
        x, y, w, h = roi1
        dst = dst[0:h, 0:w, :]

        markerCorners, markerIds, _ = arucoDetector.detectMarkers(dst)

        if markerIds is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 240, K1, None)
            for i, markerId in enumerate(markerIds):
                if markerId == 9:
                    dst = cv2.aruco.drawDetectedMarkers(dst, np.array([markerCorners[i]], dtype=np.float32), np.array([markerId]))
                    dst = cv2.drawFrameAxes(dst, K1, None, rvecs[i], tvecs[i], 1000)

        video_writer.write(dst)

        cv2.imshow("frame", dst)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):
            print(f"[INFO] Gravação encerrada e salva como '{output_path}'")
            break
        elif key == ord('q'):
            print("[INFO] Encerrando...")
            video_writer.release()
            cv2.destroyAllWindows()
            exit()

    video_writer.release()

# Setup
camera_id = 1
broker_uri = "amqp://rabbitmq:30000"
channel = Channel(broker_uri)
subscription = Subscription(channel=channel)
subscription.subscribe(topic='CameraGateway.{}.Frame'.format(camera_id))

dados1 = np.load('/pasta/calib_rt1.npz')
RT1 = dados1['rt']
K1 = dados1['K']
dist1 = dados1['dist']
nK1 = dados1['nK']
roi1 = dados1['roi']
undistort_params = (K1, dist1, nK1, roi1)

parameters = cv2.aruco.DetectorParameters()
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
arucoDetector = cv2.aruco.ArucoDetector(dictionary, parameters)

output_dir = "./output_media"
os.makedirs(output_dir, exist_ok=True)
video_output_path = os.path.join(output_dir, "output_video.avi")

recording = False
frame_size = None
fps = 20
frame_index = 1

while True:
    msg = channel.consume()
    if type(msg) != bool:
        img = msg.unpack(Image)
        frame = to_np(img)

    dst = cv2.undistort(frame, K1, dist1, None, nK1)
    x, y, w, h = roi1
    dst = dst[0:h, 0:w, :]

    if frame_size is None:
        frame_size = (dst.shape[1], dst.shape[0])

    markerCorners, markerIds, _ = arucoDetector.detectMarkers(dst)

    if markerIds is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 240, nK1, None)

        for i, markerId in enumerate(markerIds):
            if markerId == 9:
                dst = cv2.aruco.drawDetectedMarkers(dst, np.array([markerCorners[i]], dtype=np.float32), np.array([markerId]))
                dst = cv2.drawFrameAxes(dst, nK1, None, rvecs[i], tvecs[i], 1000)

                transform_translation_x = tvecs[i][0][0]
                transform_translation_y = tvecs[i][0][1]
                transform_translation_z = tvecs[i][0][2]

                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()

                roll_x, pitch_y, yaw_z = euler_from_quaternion(*quat)
                roll_x = math.degrees(roll_x)
                pitch_y = math.degrees(pitch_y)
                yaw_z = math.degrees(yaw_z)

                print("transform_translation_x: {}".format(transform_translation_x))
                print("transform_translation_y: {}".format(transform_translation_y))
                print("transform_translation_z: {}".format(transform_translation_z))
                print("roll_x: {}".format(roll_x))
                print("pitch_y: {}".format(pitch_y))
                print("yaw_z: {}".format(yaw_z))
                print()

    cv2.imshow('frame', dst)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        save_frame_image(dst.copy(), frame_index, output_dir)
        frame_index += 1

    elif key == ord('r'):
        record_video_loop(video_output_path, frame_size, fps, channel, subscription, undistort_params)

    elif key == ord('q'):
        print("[INFO] Encerrando...")
        break

cv2.destroyAllWindows()
