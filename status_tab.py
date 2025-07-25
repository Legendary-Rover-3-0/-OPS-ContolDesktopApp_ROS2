import sys
import subprocess
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QTextEdit, QLabel, QListWidget, QHBoxLayout, QComboBox, QDialog
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QRunnable, QThreadPool, QObject
import config
import os
os.environ['ANSIBLE_PYTHON_WARNINGS'] = 'False'

# Nowa klasa QObject dla sygnałów
class AnsibleSignals(QObject):
    output = pyqtSignal(str)
    finished = pyqtSignal()
    error = pyqtSignal(str)

# Wątek do wykonywania zadań Ansible w tle (QRunnable)
class AnsibleTask(QRunnable):
    def __init__(self, command, signals):
        super().__init__()
        self.command = command
        self.signals = signals # Przekazujemy instancję sygnałów

    def run(self):
        try:
            process = subprocess.Popen(self.command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = process.communicate()
            if stdout:
                self.signals.output.emit(stdout.decode())
            if stderr:
                self.signals.output.emit(stderr.decode())
        except Exception as e:
            self.signals.output.emit(f"Błąd wykonania Ansible: {e}")
            self.signals.error.emit(str(e))
        finally:
            self.signals.finished.emit() # Zawsze emituj sygnał finished na koniec

# Klasa StatusTab (zawierająca GUI i logikę Ansible)
class StatusTab(QWidget):
    def __init__(self):
        super().__init__()
        self.threadpool = QThreadPool()
        self.inventory_path = config.ANSIBLE_INVENTORY
        # self.buttons = [] # Nadal usunięte
        self.active_ansible_tasks = 0 # Licznik aktywnych zadań Ansible
        # self.current_ansible_task = None # <<< USUNIĘTO - nie jest już potrzebne
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        
        # Wybor komunikacji
        self.group_selector = QComboBox()
        self.group_selector.addItems(["rover_ubiquiti", "rover_wifi"])
        layout.addWidget(QLabel("Wybierz grupę hostów:"))
        layout.addWidget(self.group_selector)
        layout.addSpacing(10)

        #Listy portow i screenow
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

        self.unplug_and_plug_hub_button = QPushButton("⛔ Odłącz Huba")
        self.unplug_and_plug_hub_button.clicked.connect(lambda: self.set_gpio_state(0)) 
        left_column.addWidget(self.unplug_and_plug_hub_button)

        self.unplug_and_plug_hub_button = QPushButton("✅ Podłącz Huba")
        self.unplug_and_plug_hub_button.clicked.connect(lambda: self.set_gpio_state(1)) 
        left_column.addWidget(self.unplug_and_plug_hub_button)

        self.auto_stop_button = QPushButton("⏹️ AutoStop")
        self.auto_stop_button.clicked.connect(self.auto_stop_callback)  
        left_column.addWidget(self.auto_stop_button)

        left_column.addStretch()

        # Druga kolumna przycisków
        center_column = QVBoxLayout()
        self.start_satel = QPushButton("📻 Uruchom SATEL Decoder")
        self.start_satel.clicked.connect(self.start_satel_callback)
        center_column.addWidget(self.start_satel)

        self.start_science_backup = QPushButton("🧪 Uruchom Science Backup")
        self.start_science_backup.clicked.connect(self.start_science_backup_callback)
        center_column.addWidget(self.start_science_backup)

        self.show_ports_details = QPushButton("📋 Pokaz porty szeregowe")
        self.show_ports_details.clicked.connect(self.show_ports_details_callback)
        center_column.addWidget(self.show_ports_details)

        self.ledy_tur_tur = QPushButton("🪩💃🕺🎶 Ledy Tür Tür")
        self.ledy_tur_tur.clicked.connect(self.ledy_tur_tur_callback)
        center_column.addWidget(self.ledy_tur_tur)

        center_column.addStretch()

        # self.empty_button = QPushButton("            ")
        # center_column.addWidget(self.empty_button)

        # Trzecia kolumna przyciskow
        right_column = QVBoxLayout()
        self.start_autonomy_button = QPushButton("🤖 Autonomia Baza")
        self.start_autonomy_button.clicked.connect(self.start_autonomy)
        right_column.addWidget(self.start_autonomy_button)

        self.start_autonomy_script1_button = QPushButton("🏃 Autonomia Tunel")
        self.start_autonomy_script1_button.clicked.connect(self.start_autonomy_script1)
        right_column.addWidget(self.start_autonomy_script1_button)

        self.start_gps = QPushButton("🛰️ Uruchom GPS")
        self.start_gps.clicked.connect(self.start_gps_callback)
        right_column.addWidget(self.start_gps)

        self.magnetometr_button = QPushButton("🧭 Magnetometr")
        self.magnetometr_button.clicked.connect(self.magnetometr_callback)
        right_column.addWidget(self.magnetometr_button)

        right_column.addStretch()

        # Dodanie kolumn do kontenera
        buttons_layout.addLayout(left_column)
        buttons_layout.addLayout(center_column)
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

        self.stop_screen_button = QPushButton("🛑 Zatrzymaj wybrany screen (CRTL C)")
        self.stop_screen_button.clicked.connect(self.stop_screen)
        screens_layout.addWidget(self.stop_screen_button)

        self.hard_stop_screen_button = QPushButton("☠️ Zabij proces w screenie (w ostatecznosci)")
        self.hard_stop_screen_button.clicked.connect(self.hard_stop_screen)
        screens_layout.addWidget(self.hard_stop_screen_button)

        # self.wipe_dead_button = QPushButton("🗑️ Usuń martwe procesy")
        # self.wipe_dead_button.clicked.connect(self.wipe_dead_sceens)
        # screens_layout.addWidget(self.wipe_dead_button)

        list_layout.addLayout(ports_layout)
        list_layout.addSpacing(10)
        list_layout.addLayout(screens_layout)
        layout.addLayout(list_layout)
        layout.addSpacing(20)

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

        layout.addLayout(vision_buttons_layout)
        layout.addLayout(stop_vision_buttons_layout)

        self.output_area = QTextEdit()
        self.output_area.setReadOnly(True)
        layout.addWidget(self.output_area)

        self.setLayout(layout)
        self.setWindowTitle("Zarządzanie agentami")

    def run_vision_script(self, cam):
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
        commands = {
            1: config.CAMERA_1_HANDLE,
            2: config.CAMERA_2_HANDLE,
            3: config.CAMERA_3_HANDLE,
            4: config.CAMERA_4_HANDLE,
        }

        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a '"
            f"fuser -k {commands[cam]}; "
            f"screen -S camera{cam} -X quit'"
            f"",
            callback=self.view_screens
        )

    def get_selected_group(self):
        return self.group_selector.currentText()

    def get_ports(self):
        self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a '/home/legendary/kubatk/list_ports_jetson.py'")

    def start_screen(self, name_scrypt):
        selected = self.port_list.currentItem()
        if selected:
            selected_text = selected.text().split(" ")[0]
            self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS {selected_text.replace('/dev/', '')} {name_scrypt} {selected_text}'", callback=self.view_screens)
        else:
            self.output_area.append("Wybierz port z listy, aby uruchomić screen.")

    def view_screens(self):
        self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -ls'")

    def stop_screen(self):
        selected = self.screen_list.currentItem()
        if selected:
            screen_name = selected.text().split('.')[0].strip()
            #self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -S {screen_name} -X quit'", callback=self.view_screens)
            self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -S {screen_name} -X stuff $\"\\003\"'", callback=self.view_screens)

        else:
            self.output_area.append("Wybierz screen z listy, aby go zatrzymać.")

    def hard_stop_screen(self):
        selected = self.screen_list.currentItem()
        if selected:
            screen_name = selected.text().split('.')[0].strip()
            self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -S {screen_name} -X quit'", callback=self.view_screens)
            #self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -S {screen_name} -X stuff $\"\\003\"'", callback=self.view_screens)

        else:
            self.output_area.append("Wybierz screen z listy, aby go zatrzymać.")

    def start_autonomy(self):
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS AutonomyBase {config.AUTONOMY_BASE_SCRIPT}'",
            callback=self.view_screens
        )

    def start_autonomy_script1(self):
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS AutonomyTunel {config.AUTONOMY_SCRIPT1}'",
            callback=self.view_screens
        )

    def start_gps_callback(self):
        selected = self.port_list.currentItem()
        if selected:
            selected_text = selected.text().split(" ")[0]
            self.run_ansible(
                f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS GPS_{selected_text.replace('/dev/', '')} {config.START_GPS_SCRIPT} {selected_text}'"
            )
            self.run_ansible(
                f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS GPS_magnetometr {config.START_MAGNETOMETR}'"
            )
            self.run_ansible(
                f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS GPS_targets_to_yaml {config.START_TARGETS_TO_YAML}'",
                callback=self.view_screens
            )
        else:
            self.output_area.append("Wybierz port z listy, aby uruchomić GPS.")

    def start_satel_callback(self):
        selected = self.port_list.currentItem()
        if selected:
            selected_text = selected.text().split(" ")[0]
            self.run_ansible(
                f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS SATEL_{selected_text.replace('/dev/', '')} {config.START_SATEL_DECODER} {selected_text}'",
                callback=self.view_screens
            )
        else:
            self.output_area.append("Wybierz port z listy, aby uruchomić SATEL Decoder.")

    def start_science_backup_callback(self):
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS Science_backup {config.START_SCIENCE_BACKUP}'",
            callback=self.view_screens
        )

    def ledy_tur_tur_callback(self):
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS Ledy {config.START_LEDY_TUR_TUR}'",
            callback=self.view_screens
        )

    def show_ports_details_callback(self):
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a '{config.PORT_DETAILS_PY_SCRIPT}'",
            output_func=self.show_logs
        )

    def set_gpio_state(self, state):
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a '{config.GPIO_RESET} {state}'",
            callback=self.view_screens
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

    def magnetometr_callback(self):
        self.run_ansible(
            f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -dmS Magnetometr {config.START_MAGNETOMETR}'",
            callback=self.view_screens
        )


    def fetch_logs(self):
        selected = self.screen_list.currentItem()
        if selected:
            screen_name = selected.text().split('.')[0].strip()
            self.run_ansible(
                f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -S {screen_name} -X hardcopy -h /tmp/{screen_name}_log && tail -n 200 /tmp/{screen_name}_log'",
                output_func=self.show_logs
            )
        else:
            self.output_area.append("Wybierz screen z listy, aby pobrać logi.")
            
    def show_logs(self, text):
        dialog = QDialog(self)
        dialog.setWindowTitle(f"Logi agenta")
        dialog.resize(600, 400)
        
        dialog_layout = QVBoxLayout()
        
        log_viewer = QTextEdit()
        log_viewer.setReadOnly(True)
        log_viewer.setPlainText(text)
        log_viewer.setMinimumSize(580, 380)
        log_viewer.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        
        dialog_layout.addWidget(log_viewer)
        dialog.setLayout(dialog_layout)
        dialog.exec()

    # def wipe_dead_sceens(self):
    #     self.run_ansible(f"ansible -i {self.inventory_path} {self.get_selected_group()} -m shell -a 'screen -wipe'")

    def display_output(self, text):
        self.output_area.append(text)
        
        if "/dev/tty" in text:
            self.port_list.clear()
            ports = [p.strip() for p in text.strip().split("\n") if p.startswith("/dev/")]
            self.port_list.addItems(ports)
        
        if "Attached" in text or "Detached" in text or "No Sockets found" in text:
            self.screen_list.clear()
            screens = []
            for line in text.strip().split("\n"):
                if "tached" in line:
                    parts = line.split('\t')
                    if len(parts) > 1:
                        screen_info = parts[1].strip()
                        screens.append(screen_info)
            self.screen_list.addItems(screens)
            
        if "No Sockets found" in text:
            self.screen_list.clear()

    def run_ansible(self, command, callback=None, output_func=None):
        # Zwiększamy licznik aktywnych zadań
        self.active_ansible_tasks += 1
        # Ustawiamy kursor ładowania, jeśli to pierwsze aktywne zadanie
        if self.active_ansible_tasks == 1:
            QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        
        if output_func is None:
            output_func = self.display_output

        ansible_signals = AnsibleSignals()
        ansible_signals.output.connect(output_func)
        ansible_signals.finished.connect(lambda: self.on_ansible_task_finished(callback))
        ansible_signals.error.connect(lambda msg: self.output_area.append(f"Błąd zadania: {msg}"))

        task = AnsibleTask(command, ansible_signals)
        self.threadpool.start(task)

    def on_ansible_task_finished(self, callback):
        # Zmniejszamy licznik aktywnych zadań
        self.active_ansible_tasks -= 1
        
        # Przywracamy domyślny kursor tylko, gdy wszystkie zadania się skończyły
        if self.active_ansible_tasks == 0:
            QApplication.restoreOverrideCursor()
        
        if callback:
            callback()

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
