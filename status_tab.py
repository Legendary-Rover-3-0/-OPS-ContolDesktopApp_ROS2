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
        
        self.group_selector = QComboBox()
        self.group_selector.addItems(["rover_ubiquiti", "rover_wifi"])
        layout.addWidget(QLabel("Wybierz grupę hostów:"))
        layout.addWidget(self.group_selector)

        self.label_ports = QLabel("Lista portów szeregowych:")
        self.label_screens = QLabel("Lista aktywnych screenów:")

        list_layout = QHBoxLayout()
        self.port_list = QListWidget()
        self.screen_list = QListWidget()

        list_layout.addWidget(self.port_list)
        list_layout.addWidget(self.screen_list)

        layout.addWidget(self.label_ports)
        layout.addWidget(self.label_screens)
        layout.addLayout(list_layout)

        self.refresh_button = QPushButton("Odśwież listę portów")
        self.refresh_button.clicked.connect(self.get_ports)
        layout.addWidget(self.refresh_button)

        #guziki start sceenów

        start_screen_buttons_layout = QHBoxLayout()
        
        self.start_drive_screen_button = QPushButton("Jazda 💨")
        self.start_drive_screen_button.clicked.connect(lambda _: self.start_screen(config.AGENT_START_SCRIPT))
        start_screen_buttons_layout.addWidget(self.start_drive_screen_button)

        self.start_mani_screen_button = QPushButton("Manipulator 🦾")
        self.start_mani_screen_button.clicked.connect(lambda _: self.start_screen(config.AGENT_START_SCRIPT))
        start_screen_buttons_layout.addWidget(self.start_mani_screen_button)

        self.start_science_screen_button = QPushButton("Science 🧪")
        self.start_science_screen_button.clicked.connect(lambda _: self.start_screen(config.AGENT_START_SCRIPT))
        start_screen_buttons_layout.addWidget(self.start_science_screen_button)

        self.start_RFID_screen_button = QPushButton("RFID 💳")
        self.start_RFID_screen_button.clicked.connect(lambda _: self.start_screen(config.AGENT_START_SCRIPT))
        start_screen_buttons_layout.addWidget(self.start_RFID_screen_button)

        self.start_GPS_screen_button = QPushButton("GPS 🛰️")
        self.start_GPS_screen_button.clicked.connect(lambda _: self.start_screen(config.AGENT_START_SCRIPT))
        start_screen_buttons_layout.addWidget(self.start_GPS_screen_button)

        layout.addLayout(start_screen_buttons_layout) 


        #guziki sceeny
        screen_buttons_layout = QHBoxLayout()

        self.view_screens_button = QPushButton("Odśwież listę screenów")
        self.view_screens_button.clicked.connect(self.view_screens)
        screen_buttons_layout.addWidget(self.view_screens_button)

        self.stop_screen_button = QPushButton("Zatrzymaj wybrany screen")
        self.stop_screen_button.clicked.connect(self.stop_screen)
        screen_buttons_layout.addWidget(self.stop_screen_button)

        self.fetch_logs_button = QPushButton("Pobierz logi z wybranego screena")
        self.fetch_logs_button.clicked.connect(self.fetch_logs)
        screen_buttons_layout.addWidget(self.fetch_logs_button)

        self.wipe_dead_button = QPushButton("Usuń martwe procesy")
        self.wipe_dead_button.clicked.connect(self.wipe_dead_sceens)
        screen_buttons_layout.addWidget(self.wipe_dead_button)

        layout.addLayout(screen_buttons_layout) 

        # Guziki wizja
        vision_buttons_layout = QHBoxLayout()

        self.run_vision_script_button = QPushButton("Uruchom skrypt wizji")
        self.run_vision_script_button.clicked.connect(self.run_vision_script)
        vision_buttons_layout.addWidget(self.run_vision_script_button)

        self.fetch_vision_logs_button = QPushButton("Pobierz logi wizji")
        self.fetch_vision_logs_button.clicked.connect(self.fetch_vision_logs)
        vision_buttons_layout.addWidget(self.fetch_vision_logs_button)

        self.stop_vision_screen_button = QPushButton("Zatrzymaj proces wizji")
        self.stop_vision_screen_button.clicked.connect(self.stop_vision_screen)
        vision_buttons_layout.addWidget(self.stop_vision_screen_button)

        layout.addLayout(vision_buttons_layout)  # Dodanie poziomego layoutu do głównego layoutu


        self.output_area = QTextEdit()
        self.output_area.setReadOnly(True)
        layout.addWidget(self.output_area)

        self.setLayout(layout)
        self.setWindowTitle("Zarządzanie agentami")

    def run_vision_script(self):
        """Uruchamia skrypt wizji na zdalnym hoście."""
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS wizja {config.VISION_SCRIPT}'"
        )

    def fetch_vision_logs(self):
        """Pobiera logi z ekranu wizja."""
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -S wizja -X hardcopy -h /tmp/wizja_log && tail -n 200 /tmp/wizja_log'",
            output=self.show_logs
        )

    def stop_vision_screen(self):
        """Zatrzymuje proces w ekranie wizja poprzez wysłanie Ctrl+C."""
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -S wizja -X stuff \"\\003\"'"
        )


    def get_selected_group(self):
        return self.group_selector.currentText()

    def get_ports(self):
        self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'find /dev/ -maxdepth 1 -type c \( -name ttyS\* -o -name ttyUSB\* -o -name ttyA\* \)'")

    def start_screen(self, name_scrypt):
        selected = self.port_list.currentItem()
        if selected:
            self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS {selected.text().replace('/dev/', '')} {name_scrypt} {selected.text()}'", callback=self.view_screens)

    def view_screens(self):
        self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -ls'")

    def stop_screen(self):
        selected = self.screen_list.currentItem()
        if selected:
            screen_name = selected.text().split('.')[0]
            self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -S {screen_name} -X quit'", callback=self.view_screens)

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

# Kod uruchamiający aplikację
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = StatusTab()
    window.show()
    sys.exit(app.exec())
