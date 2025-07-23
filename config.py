# Tematy ROS
GAMEPAD_TOPIC = 'gamepad_input'
BUTTON_TOPIC = 'button_states'

# Kolory przycisków
BUTTON_ON_COLOR = "#2ECC71"
BUTTON_OFF_COLOR = "#FF5733"
BUTTON_DEFAULT_COLOR = "#3a3a3a"
BUTTON_SELECTED_COLOR = "blue"

# Konfiguracja agentów
ANSIBLE_INVENTORY = "ansible_inventory.ini"
AGENT_START_SCRIPT = "/home/legendary/kubatk/drive.sh" # skrypt musi pobierać 1 parametr (np: /dev/ttyUSB0)
UNPLUG_PLUG_SCRIPT = "/home/legendary/kubatk/unplug.sh" #skrypt musi pobierać 1 parametr

AUTONOMY_BASE_SCRIPT = "/home/legendary/kubatk/autonomy.sh" # Baza: lidar i te inne
AUTONOMY_SCRIPT1 = "/home/legendary/kubatk/autonomyTunel.sh" # Tunel
#AUTONOMY_DRIVE_SCRIPT = "/home/legendary/kubatk/autonomydrive.sh"
#AUTONOMY_DRIVE_SCRIPT = "/home/legendary/kubatk/test.sh"

START_GPS_SCRIPT = "/home/legendary/kubatk/gps.sh" # potrzebny parametr (port GPS)
START_TARGETS_TO_YAML = "/home/legendary/kubatk/gps_targets.sh" # TODO
START_SATEL_DECODER = "/home/legendary/kubatk/start_satel.sh" # potrzebny parametr port # TODO
START_SCIENCE_BACKUP = "" # TODO wrzuci ale gotowy
PORT_DETAILS_PY_SCRIPT = "/home/legendary/kubatk/show_ports.sh"
START_MAGNETOMETR = "/home/legendary/kubatk/magnetometr.sh"
AUTOSTOP_SCRIPT = "/home/legendary/Legendary_Wolverine/legendary_wolverine/Autostop/AutoStop.sh"
GPIO_RESET = "/home/legendary/kubatk/gpio/turecki_unplug.sh"

# Manipulator
MANI_DEFAULT_VALUE = 100.0

# Wizja
CAM_RECIVER_IP = "192.168.2.10"

CAMERA_1_HANDLE = "/dev/video0"
CAMERA_2_HANDLE = "/dev/video2"
CAMERA_3_HANDLE = "/dev/video4"
CAMERA_4_HANDLE = "/dev/video6"

CAMERA_1_CMD = f"gst-launch-1.0 -e v4l2src device={CAMERA_1_HANDLE} ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! textoverlay text=\\\"Kamera 1\\\" valignment=top halignment=left font-desc=\\\"Sans, 18\\\" ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=6123"
CAMERA_2_CMD = f"gst-launch-1.0 -e v4l2src device={CAMERA_2_HANDLE} ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! textoverlay text=\\\"Kamera 2\\\" valignment=top halignment=left font-desc=\\\"Sans, 18\\\" ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=7123"
CAMERA_3_CMD = f"gst-launch-1.0 -e v4l2src device={CAMERA_3_HANDLE} ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! textoverlay text=\\\"Kamera 3\\\" valignment=top halignment=left font-desc=\\\"Sans, 18\\\" ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=8123"
CAMERA_4_CMD = f"gst-launch-1.0 -e v4l2src device={CAMERA_4_HANDLE} ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! textoverlay text=\\\"Kamera 4\\\" valignment=top halignment=left font-desc=\\\"Sans, 18\\\" ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=9123"

# CAMERA_2_CMD = f"gst-launch-1.0 -e v4l2src device={CAMERA_2_HANDLE} ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=7123"
# CAMERA_1_CMD = f"gst-launch-1.0 -e v4l2src device={CAMERA_1_HANDLE} ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=6123"
# CAMERA_3_CMD = f"gst-launch-1.0 -e v4l2src device={CAMERA_3_HANDLE} ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=8123"
# CAMERA_4_CMD = f"gst-launch-1.0 -e v4l2src device={CAMERA_4_HANDLE} ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=9123"


# Science
SERVO_LITTLE_OPEN_ANGLE = 140.0
SERVO_CLOSED_ANGLE = 180.0        #  0% - domyslnie   0.0 stopni
SERVO_OPEN_ANGLE = 90.0         # 50% - domyslnie  90.0 stopni
SERVO_FULL_OPEN_ANGLE = 10.0   #100% - domyslnie 180.0 stopni  #Paweł M zamocował odwrotnie i 0 jest 180 a 180 jest 0 xD

AUTO_CLOSE_SERVOS_ON_APP_START = False # wysylanie zamkniecia serv science przy wlaczeniu apki
AUTO_CLOSE_SERVOS_ON_APP_CLOSE = False # wysylanie zamkniecia serv science przy zamknieciu apki

