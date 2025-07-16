import sys
import subprocess
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QTextEdit, QLabel, QListWidget, QHBoxLayout, QComboBox, QDialog
from PyQt6.QtCore import Qt, QThread, pyqtSignal
import config
import os
os.environ['ANSIBLE_PYTHON_WARNINGS'] = 'False'

# Wątek do wykonywania zadań Ansible w tle
class AnsibleThread(QThread):
    output = pyqtSignal(str)

    def __init__(self, command):
        super().__init__()
        self.command = command

    def run(self):
        process = subprocess.Popen(self.command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate()
        if stdout:
            self.output.emit(stdout.decode())
        if stderr:
            self.output.emit(stderr.decode())

# Klasa StatusTab (zawierająca GUI i logikę Ansible)
class StatusTab(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.threads = []  # Lista do przechowywania aktywnych wątków
        self.inventory_path = config.ANSIBLE_INVENTORY  # Ścieżka do pliku inventory

    def init_ui(self):
        layout = QVBoxLayout()
        
        # Wybor komunikacji
        self.group_selector = QComboBox()
        self.group_selector.addItems(["rover_ubiquiti", "rover_wifi"])
        layout.addWidget(QLabel("Wybierz grupę hostów:"))
        layout.addWidget(self.group_selector)
        layout.addSpacing(10)

        #Listy portow i screenow

            #Porty
        list_layout = QHBoxLayout()

        ports_layout = QVBoxLayout()
        self.label_ports = QLabel("Lista portów szeregowych:")
        self.port_list = QListWidget()
        self.port_list.setMinimumHeight(150)
        ports_layout.addWidget(self.label_ports)
        ports_layout.addWidget(self.port_list)
        self.refresh_button = QPushButton("🔄 Odśwież listę portów")
        self.refresh_button.clicked.connect(self.get_ports)
        ports_layout.addWidget(self.refresh_button)

                
        self.start_agent_screen_button = QPushButton("🚀 Uruchom Agent MicroROS")
        self.start_agent_screen_button.clicked.connect(lambda _: self.start_screen(config.AGENT_START_SCRIPT))
        ports_layout.addWidget(self.start_agent_screen_button)

        # Tworzymy kontener na przyciski w dwóch kolumnach
        buttons_container = QWidget()
        buttons_layout = QHBoxLayout()
        buttons_container.setLayout(buttons_layout)

        # Pierwsza kolumna przycisków

        left_column = QVBoxLayout()
        self.unplug_and_plug_button = QPushButton("🔌 Zresetuj Wszytkie Porty")
        self.unplug_and_plug_button.clicked.connect(self.reset_everything)
        left_column.addWidget(self.unplug_and_plug_button)

        self.start_autonomy_button = QPushButton("🤖 Uruchom autonomie (baza)")
        self.start_autonomy_button.clicked.connect(self.start_autonomy)
        left_column.addWidget(self.start_autonomy_button)

        self.start_gps = QPushButton("🛰️ Uruchom GPS")
        self.start_gps.clicked.connect(self.start_gps_callback)
        left_column.addWidget(self.start_gps)

        self.auto_stop_button = QPushButton("⏹️ AutoStop")
        self.auto_stop_button.clicked.connect(self.auto_stop_callback) 
        left_column.addWidget(self.auto_stop_button)

        # Druga kolumna przycisków
        right_column = QVBoxLayout()
        self.start_satel = QPushButton("📻 Uruchom SATEL Decoder")
        self.start_satel.clicked.connect(self.start_satel_callback)
        right_column.addWidget(self.start_satel)

        self.start_science_backup = QPushButton("🧪 Uruchom Science Backup")
        self.start_science_backup.clicked.connect(self.start_science_backup_callback)
        right_column.addWidget(self.start_science_backup)

        self.show_ports_details = QPushButton("📋 Pokaz porty szeregowe")
        self.show_ports_details.clicked.connect(self.show_ports_details_callback)
        right_column.addWidget(self.show_ports_details)

        self.empty_button = QPushButton("                  ")
        #self.empty_butto.clicked.connect(self.auto_stop_callback)
        right_column.addWidget(self.empty_button)

        # Dodanie kolumn do kontenera
        buttons_layout.addLayout(left_column)
        buttons_layout.addLayout(right_column)
        buttons_layout.addStretch()

        # Dodanie kontenera z przyciskami do głównego layoutu
        ports_layout.addWidget(buttons_container)
        ports_layout.addStretch(100)

        # Screeny
        screens_layout = QVBoxLayout()
        self.label_screens = QLabel("Lista aktywnych screenów:")
        self.screen_list = QListWidget()
        self.screen_list.setMinimumHeight(150)
        screens_layout.addWidget(self.label_screens)
        screens_layout.addWidget(self.screen_list)
        self.view_screens_button = QPushButton("🔄 Odśwież listę screenów")
        self.view_screens_button.clicked.connect(self.view_screens)
        screens_layout.addWidget(self.view_screens_button)

        self.fetch_logs_button = QPushButton("📝 Pobierz logi z wybranego screena")
        self.fetch_logs_button.clicked.connect(self.fetch_logs)
        screens_layout.addWidget(self.fetch_logs_button)

        self.stop_screen_button = QPushButton("🛑 Zatrzymaj wybrany screen")
        self.stop_screen_button.clicked.connect(self.stop_screen)
        screens_layout.addWidget(self.stop_screen_button)

        self.wipe_dead_button = QPushButton("🗑️ Usuń martwe procesy")
        self.wipe_dead_button.clicked.connect(self.wipe_dead_sceens)
        screens_layout.addWidget(self.wipe_dead_button)


        list_layout.addLayout(ports_layout)
        list_layout.addSpacing(10)
        list_layout.addLayout(screens_layout)
        layout.addLayout(list_layout)
        layout.addSpacing(20)

        

        # #guziki start sceenów

        # start_screen_buttons_layout = QHBoxLayout()
        
        # self.start_drive_screen_button = QPushButton("Jazda 💨")
        # self.start_drive_screen_button.clicked.connect(lambda _: self.start_screen(config.AGENT_START_SCRIPT))
        # start_screen_buttons_layout.addWidget(self.start_drive_screen_button)

        # self.start_mani_screen_button = QPushButton("Manipulator 🦾")
        # self.start_mani_screen_button.clicked.connect(lambda _: self.start_screen(config.AGENT_START_SCRIPT))
        # start_screen_buttons_layout.addWidget(self.start_mani_screen_button)

        # self.start_science_screen_button = QPushButton("Science 🧪")
        # self.start_science_screen_button.clicked.connect(lambda _: self.start_screen(config.AGENT_START_SCRIPT))
        # start_screen_buttons_layout.addWidget(self.start_science_screen_button)

        # self.start_RFID_screen_button = QPushButton("RFID 💳")
        # self.start_RFID_screen_button.clicked.connect(lambda _: self.start_screen(config.AGENT_START_SCRIPT))
        # start_screen_buttons_layout.addWidget(self.start_RFID_screen_button)

        # self.start_GPS_screen_button = QPushButton("GPS 🛰️")
        # self.start_GPS_screen_button.clicked.connect(lambda _: self.start_screen(config.AGENT_START_SCRIPT))
        # start_screen_buttons_layout.addWidget(self.start_GPS_screen_button)

        # layout.addLayout(start_screen_buttons_layout) 


        # #guziki sceeny
        # screen_buttons_layout = QHBoxLayout()

        # self.stop_screen_button = QPushButton("Zatrzymaj wybrany screen")
        # self.stop_screen_button.clicked.connect(self.stop_screen)
        # screen_buttons_layout.addWidget(self.stop_screen_button)

        # self.fetch_logs_button = QPushButton("Pobierz logi z wybranego screena")
        # self.fetch_logs_button.clicked.connect(self.fetch_logs)
        # screen_buttons_layout.addWidget(self.fetch_logs_button)

        # self.wipe_dead_button = QPushButton("Usuń martwe procesy")
        # self.wipe_dead_button.clicked.connect(self.wipe_dead_sceens)
        # screen_buttons_layout.addWidget(self.wipe_dead_button)

        # layout.addLayout(screen_buttons_layout) 

        # Guziki wizja
        vision_buttons_layout = QHBoxLayout()

        self.run_vision_script_button1 = QPushButton("🎦 Włącz kamere 1")
        self.run_vision_script_button1.clicked.connect(lambda _: self.run_vision_script(1))
        vision_buttons_layout.addWidget(self.run_vision_script_button1)

        self.run_vision_script_button2 = QPushButton("🎦 Włącz kamere 2")
        self.run_vision_script_button2.clicked.connect(lambda _: self.run_vision_script(2))
        vision_buttons_layout.addWidget(self.run_vision_script_button2)

        self.run_vision_script_button3 = QPushButton("🎦 Włącz kamere 3")
        self.run_vision_script_button3.clicked.connect(lambda _: self.run_vision_script(3))
        vision_buttons_layout.addWidget(self.run_vision_script_button3)

        self.run_vision_script_button4 = QPushButton("🎦 Włącz kamere 4")
        self.run_vision_script_button4.clicked.connect(lambda _: self.run_vision_script(4))
        vision_buttons_layout.addWidget(self.run_vision_script_button4)

        # Guziki stop wizja
        stop_vision_buttons_layout = QHBoxLayout()

        self.stop_vision_script_button1 = QPushButton("❌ Wyłącz kamere 1")
        self.stop_vision_script_button1.clicked.connect(lambda _: self.stop_vision_script(1))
        stop_vision_buttons_layout.addWidget(self.stop_vision_script_button1)

        self.stop_vision_script_button2 = QPushButton("❌ Wyłącz kamere 2")
        self.stop_vision_script_button2.clicked.connect(lambda _: self.stop_vision_script(2))
        stop_vision_buttons_layout.addWidget(self.stop_vision_script_button2)

        self.stop_vision_script_button3 = QPushButton("❌ Wyłącz kamere 3")
        self.stop_vision_script_button3.clicked.connect(lambda _: self.stop_vision_script(3))
        stop_vision_buttons_layout.addWidget(self.stop_vision_script_button3)

        self.stop_vision_script_button4 = QPushButton("❌ Wyłącz kamere 4")
        self.stop_vision_script_button4.clicked.connect(lambda _: self.stop_vision_script(4))
        stop_vision_buttons_layout.addWidget(self.stop_vision_script_button4)


        layout.addLayout(vision_buttons_layout)  # Dodanie poziomego layoutu do głównego layoutu
        layout.addLayout(stop_vision_buttons_layout)  # Dodanie poziomego layoutu do głównego layoutu


        self.output_area = QTextEdit()
        self.output_area.setReadOnly(True)
        layout.addWidget(self.output_area)

        self.setLayout(layout)
        self.setWindowTitle("Zarządzanie agentami")

    def run_vision_script(self, cam):
        """Uruchamia skrypt w screenie z automatycznym restartem"""
        commands = {
            1: f"while true; do {config.CAMERA_1_CMD}; sleep 1; done",
            2: f"while true; do {config.CAMERA_2_CMD}; sleep 1; done", 
            3: f"while true; do {config.CAMERA_3_CMD}; sleep 1; done",
            4: f"while true; do {config.CAMERA_4_CMD}; sleep 1; done"
        }
    
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS camera{cam} bash -c \"{commands[cam]}\"'",
            callback=self.view_screens
        )

    def stop_vision_script(self, cam):
        """Zatrzymuje skrypt wizji i zamyka powiązany screen"""
        commands = {
            1: config.CAMERA_1_HANDLE,
            2: config.CAMERA_2_HANDLE,
            3: config.CAMERA_3_HANDLE,
            4: config.CAMERA_4_HANDLE,
        }

        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a '"
            f"fuser -k {commands[cam]}; "  # Zabija proces GStreamera
            f"screen -S camera{cam} -X quit'"  # Zamyka screen
            f"",
            callback=self.view_screens
        )


    # def fetch_vision_logs(self):
    #     """Pobiera logi z ekranu wizja."""
    #     self.run_ansible(
    #         f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -S wizja -X hardcopy -h /tmp/wizja_log && tail -n 200 /tmp/wizja_log'",
    #         output=self.show_logs
    #     )

    # def stop_vision_screen(self):
    #     """Zatrzymuje proces w ekranie wizja poprzez wysłanie Ctrl+C."""
    #     self.run_ansible(
    #         f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -S wizja -X stuff \"\\003\"'"
    #     )


    def get_selected_group(self):
        return self.group_selector.currentText()

    def get_ports(self):
        #self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'find /dev/ -maxdepth 1 -type c \( -name ttyS\* -o -name ttyUSB\* -o -name ttyA\* \)'")
        # self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'find /dev/ -maxdepth 1 -type c \( -name ttyUSB\* -o -name ttyAC\* \)'")
        self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a '/home/legendary/kubatk/list_ports_jetson.py'")

    def start_screen(self, name_scrypt):
        selected = self.port_list.currentItem()
        selected = selected.text().split(" ")
        selected = selected[0]
        if selected:
            self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS {selected.replace('/dev/', '')} {name_scrypt} {selected}'", callback=self.view_screens)

    def view_screens(self):
        self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -ls'")

    def stop_screen(self):
        selected = self.screen_list.currentItem()
        if selected:
            screen_name = selected.text().split('.')[0]
            self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -S {screen_name} -X quit'", callback=self.view_screens)

    def start_autonomy(self):
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS AutonomyBase {config.AUTONOMY_BASE_SCRIPT}'",
            callback=self.view_screens
        )

    def start_gps_callback(self):
        selected = self.port_list.currentItem()
        selected = selected.text().split(" ")
        selected = selected[0]
        if selected:
            self.run_ansible(
                f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS GPS_{selected.replace('/dev/', '')} {config.START_GPS_SCRIPT} {selected}'"
            )
            self.run_ansible(
                f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS GPS_magnetometr {config.START_MAGNETOMETR}'"
            )
            self.run_ansible(
                f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS GPS_targets_to_yaml {config.START_TARGETS_TO_YAML}'",
                callback=self.view_screens
            )

    def start_satel_callback(self):
        selected = self.port_list.currentItem()
        selected = selected.text().split(" ")
        selected = selected[0]
        if selected:
            self.run_ansible(
                f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS SATEL_{selected.replace('/dev/', '')} {config.START_SATEL_DECODER} {selected}'",
                callback=self.view_screens
            )

    def start_science_backup_callback(self):
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS Science_backup {config.START_SCIENCE_BACKUP}'",
            callback=self.view_screens
        )

    def show_ports_details_callback(self):
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a '{config.PORT_DETAILS_PY_SCRIPT}'",
            output=self.show_logs
        )
    
    def start_autonomy_drive(self):
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS AutonomyDrive {config.AUTONOMY_DRIVE_SCRIPT}'",
            callback=self.view_screens
        )

    def stop_autonomy_drive(self):
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -S AutonomyDrive -X quit'",
            callback=self.view_screens
        )

    def auto_stop_callback(self):
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS AutoStop {config.AUTOSTOP_SCRIPT}'",
            callback=self.view_screens
        )

    def fetch_logs(self):
        selected = self.screen_list.currentItem()
        if selected:
            screen_name = selected.text().split('.')[0]
            self.run_ansible(
                f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -S {screen_name} -X hardcopy -h /tmp/{screen_name}_log && tail -n 200 /tmp/{screen_name}_log'",
                output=self.show_logs
            )
            
    def show_logs(self, text):
        dialog = QDialog(self)
        dialog.setWindowTitle(f"Logi agenta")
        dialog.resize(600, 400)  # Powiększone okno
        
        dialog_layout = QVBoxLayout()
        
        log_viewer = QTextEdit()
        log_viewer.setReadOnly(True)
        log_viewer.setPlainText(text)
        log_viewer.setMinimumSize(580, 380)  # Ograniczenie minimalnego rozmiaru
        log_viewer.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        
        dialog_layout.addWidget(log_viewer)
        dialog.setLayout(dialog_layout)
        dialog.exec()

    def wipe_dead_sceens(self):
        self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -wipe'")


    def cleanup_thread(self, thread, callback):
        if thread in self.threads:
            self.threads.remove(thread)  # Usunięcie zakończonego wątku
        if callback:
            callback()  # Jeśli przekazano callback, wykonaj go po zakończeniu wątku

    def display_output(self, text):
        self.output_area.append(text)
        
        if "/dev/tty" in text:
            self.port_list.clear()
            ports = [p for p in text.strip().split("\n") if p.startswith("/")]
            self.port_list.addItems(ports)
        
        if "Attached" in text or "Detached" in text:
            self.screen_list.clear()
            screens = [line.split('\t')[1] for line in text.strip().split("\n") if "tached" in line]
            self.screen_list.addItems(screens)

        if "No Sockets found" in text:
            self.screen_list.clear()

    def run_ansible(self, command, callback=None, output=None):
        if output is None:
            output = self.display_output  # Przekazanie metody instancji

        thread = AnsibleThread(command)
        thread.output.connect(lambda text: output(text))
        thread.finished.connect(lambda: self.cleanup_thread(thread, callback))
        self.threads.append(thread)  # Dodanie wątku do listy
        thread.start()
    
    def reset_everything(self):
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a '/home/legendary/kubatk/unplug_URC.sh' --become",
            callback=self.view_screens
        )


# Kod uruchamiający aplikację
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = StatusTab()
    window.show()
    sys.exit(app.exec())
