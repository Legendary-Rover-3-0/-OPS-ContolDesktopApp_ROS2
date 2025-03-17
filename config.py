# Tematy ROS
GAMEPAD_TOPIC = 'gamepad_input'
BUTTON_TOPIC = 'button_states'
CAMERA_TOPICS = [
    '/RUN1/transport/compressed',
    '/RUN2/transport/compressed',
    '/RUN3/transport/compressed',
    '/RUN4/transport/compressed'
    #'/laptop_webcam/transport/compressed',  # Przykładowy temat dla kamery 1
    #'/runcam/transport/compressed',         # Przykładowy temat dla kamery 2
    # Dodaj więcej tematów, jeśli potrzebujesz
]
# Kolory przycisków
BUTTON_ON_COLOR = "green"
BUTTON_OFF_COLOR = "red"

# Konfiguracja agentów
ANSIBLE_INVENTORY = "ansible_inventory.ini"
AGENT_START_SCRIPT = "/home/legendary/kubatk/test.sh" # skrypt musi pobierać 1 parametr (np: /dev/ttyUSB0)
VISION_SCRIPT = "/home/legendary/kubatk/vision.sh"

# Wizja
CAM_RECIVER_IP = "192.168.2.2"
CAMERA_1_CMD = f"gst-launch-1.0  v4l2src device=/dev/video0 ! image/jpeg,width=1920,height=1080,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=6123"
CAMERA_2_CMD = f"gst-launch-1.0  v4l2src device=/dev/video2 ! image/jpeg,width=1920,height=1080,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=7123"
CAMERA_3_CMD = f"gst-launch-1.0  v4l2src device=/dev/video4 ! image/jpeg,width=1920,height=1080,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=8123"
CAMERA_4_CMD = f"gst-launch-1.0  v4l2src device=/dev/video6 ! image/jpeg,width=1920,height=1080,framerate=30/1 ! jpegparse ! jpegdec ! videoconvert ! x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! rtph264pay ! udpsink host={CAM_RECIVER_IP} port=9123"
