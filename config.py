# Tematy ROS
GAMEPAD_TOPIC = 'gamepad_input'
BUTTON_TOPIC = 'button_states'

# Kolory przycisków
BUTTON_ON_COLOR = "green"
BUTTON_OFF_COLOR = "red"
BUTTON_DEFAULT_COLOR = "dark gray"
BUTTON_SELECTED_COLOR = "blue"

# Konfiguracja agentów
ANSIBLE_INVENTORY = "ansible_inventory.ini"
AGENT_START_SCRIPT = "/home/legendary/kubatk/drive.sh" # skrypt musi pobierać 1 parametr (np: /dev/ttyUSB0)
UNPLUG_PLUG_SCRIPT = "/home/legendary/kubatk/unplug.sh" #skrypt musi pobierać 1 parametr
AUTONOMY_BASE_SCRIPT = "/home/legendary/kubatk/autonomy.sh"
AUTONOMY_DRIVE_SCRIPT = "/home/legendary/kubatk/autonomydrive.sh"

# Manipulator
MANI_DEFAULT_VALUE = 100.0

# Wizja
CAM_RECIVER_IP = "192.168.2.10"

CAMERA_1_HANDLE = "/dev/video0"
CAMERA_2_HANDLE = "/dev/video2"
CAMERA_3_HANDLE = "/dev/video4"
CAMERA_4_HANDLE = "/dev/video6"

CAMERA_1_CMD = f"gst-launch-1.0 -e v4l2src device={CAMERA_1_HANDLE} ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=6123"
CAMERA_2_CMD = f"gst-launch-1.0 -e v4l2src device={CAMERA_2_HANDLE} ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=7123"
CAMERA_3_CMD = f"gst-launch-1.0 -e v4l2src device={CAMERA_3_HANDLE} ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=8123"
CAMERA_4_CMD = f"gst-launch-1.0 -e v4l2src device={CAMERA_4_HANDLE} ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=9123"

# Science
SERVO_CLOSED_ANGLE = 180.0        #  0% - domyslnie   0.0 stopni
SERVO_OPEN_ANGLE = 90.0         # 50% - domyslnie  90.0 stopni
SERVO_FULL_OPEN_ANGLE = 0.0   #100% - domyslnie 180.0 stopni  #Paweł M zamocował odwrotnie i 0 jest 180 a 180 jest 0 xD

AUTO_CLOSE_SERVOS_ON_APP_START = False # wysylanie zamkniecia serv science przy wlaczeniu apki
AUTO_CLOSE_SERVOS_ON_APP_CLOSE = False # wysylanie zamkniecia serv science przy zamknieciu apki

