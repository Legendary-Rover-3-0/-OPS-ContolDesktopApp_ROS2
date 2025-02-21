# Tematy ROS
GAMEPAD_TOPIC = 'gamepad_input'
BUTTON_TOPIC = 'button_states'
CAMERA_TOPICS = [
    '/dellPaw/transport/compressed',
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
