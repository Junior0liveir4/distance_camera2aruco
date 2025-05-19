from is_wire.core import Channel,Subscription,Message
from is_msgs.image_pb2 import Image
from scipy.spatial.transform import Rotation as R
import math
import numpy as np
import time
import cv2

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
  """
  Convert a quaternion into euler angles (roll, pitch, yaw)
  roll is rotation around x in radians (counterclockwise)
  pitch is rotation around y in radians (counterclockwise)
  yaw is rotation around z in radians (counterclockwise)
  """
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

  return roll_x, pitch_y, yaw_z # in radians

# Carrega o dicionario que foi usado para gerar os ArUcos e
# inicializa o detector usando valores padroes para os parametros
parameters =  cv2.aruco.DetectorParameters()
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
arucoDetector = cv2.aruco.ArucoDetector(dictionary, parameters)

camera_id = 1

# Definicoes para aquisicao das imagens no Espaco Inteligente
broker_uri = "amqp://rabbitmq:30000"
channel = Channel(broker_uri)

subscription = Subscription(channel=channel)
subscription.subscribe(topic='CameraGateway.{}.Frame'.format(camera_id))

nome_imagem = 'Camera'

dados1 = np.load('/pasta/calib_rt1.npz')
RT1 = dados1['rt']
K1 = dados1['K']
dist1 = dados1['dist']
nK1 = dados1['nK']
roi1 = dados1['roi']

recording = False
video_writer = None
frame_size = None
fps = 20

while(True):
    # Captura um frame
    msg = channel.consume()
    if type(msg) != bool:
        img = msg.unpack(Image)
        frame = to_np(img)

    # Detecta os marcadores na imagem
    # markerCorners, markerIds, rejectedImgPoints = arucoDetector.detectMarkers(frame)

    # Desenha as quinas detectadas na imagem
    # img01_corners = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
    # cv2.imshow('img01_corners',img01_corners)

    dst = cv2.undistort(frame, K1, dist1, None, nK1)
    x,y,w,h = roi1
    dst = dst[0:h, 0:w, :]
    markerCorners, markerIds, rejectedImgPoints = arucoDetector.detectMarkers(dst)

    if markerIds is not None:

      # Draw a square around detected markers in the video frame
      #cv2.aruco.drawDetectedMarkers(dst, markerCorners, markerIds)

      # Get the rotation and translation vectors
      rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 240, nK1, None)

      # Print the pose for the ArUco marker
      # The pose of the marker is with respect to the camera lens frame.
      # Imagine you are looking through the camera viewfinder,
      # the camera lens frame's:
      # x-axis points to the right
      # y-axis points straight down towards your toes
      # z-axis points straight ahead away from your eye, out of the camera
      for i, markerId in enumerate(markerIds):
        if markerId == 9:
            print(markerId)
            # Store the translation (i.e. position) information
            dst = cv2.aruco.drawDetectedMarkers(dst, np.array([markerCorners[i]],dtype=np.float32), np.array([markerId]))
            dst = cv2.drawFrameAxes(dst, nK1, None, rvecs[i], tvecs[i], 100)

            transform_translation_x = tvecs[i][0][0]
            transform_translation_y = tvecs[i][0][1]
            transform_translation_z = tvecs[i][0][2]

            # Store the rotation information
            rotation_matrix = np.eye(4)
            rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
            r = R.from_matrix(rotation_matrix[0:3, 0:3])
            quat = r.as_quat()

            # Quaternion format
            transform_rotation_x = quat[0]
            transform_rotation_y = quat[1]
            transform_rotation_z = quat[2]
            transform_rotation_w = quat[3]

            # Euler angle format in radians
            roll_x, pitch_y, yaw_z = euler_from_quaternion(transform_rotation_x,
                                                        transform_rotation_y,
                                                        transform_rotation_z,
                                                        transform_rotation_w)

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
     
    # Display the resulting frame
    cv2.imshow('frame',dst)
          
    # If "q" is pressed on the keyboard, 
    # exit this loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
