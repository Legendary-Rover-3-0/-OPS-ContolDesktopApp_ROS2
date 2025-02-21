from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QTabWidget
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap, QResizeEvent
from PyQt6.QtGui import QImage

class VisionTab(QWidget):
    def __init__(self, open_camera_window_callback):
        super().__init__()
        self.open_camera_window_callback = open_camera_window_callback
        self.init_ui()
        self.cv_images = [None] * 4  # Lista do przechowywania oryginalnych obrazów OpenCV

    def init_ui(self):
        layout = QVBoxLayout()
        self.vision_tabs = QTabWidget()
        self.camera_labels = []
        for i in range(4):
            tab = QWidget()
            tab_layout = QVBoxLayout()
            label = QLabel(f'Oczekiwanie na obraz z kamery {i + 1}...')
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Wyśrodkuj tekst
            label.setStyleSheet("background-color: black; color: white;")  # Czarny kolor tła
            label.setScaledContents(False)  # Wyłącz automatyczne skalowanie
            label.setMinimumHeight(200)  # Ustaw minimalną wysokość etykiety
            self.camera_labels.append(label)
            open_button = QPushButton(f'Otwórz kamerę {i + 1}')
            open_button.clicked.connect(lambda _, idx=i: self.open_camera_window_callback(idx))
            self.style_button(open_button, '#3498DB')  # Niebieski przycisk
            tab_layout.addWidget(label)
            tab_layout.addWidget(open_button)
            tab.setLayout(tab_layout)
            self.vision_tabs.addTab(tab, f'Kamera {i + 1}')
        layout.addWidget(self.vision_tabs)
        self.setLayout(layout)

    def style_button(self, button, color):
            button.setStyleSheet(f"""
                QPushButton {{
                    background-color: {color};
                    color: white;
                    font-weight: bold;
                    border: none;
                    padding: 10px;
                    border-radius: 5px;
                }}
                QPushButton:hover {{
                    background-color: {self.adjust_color(color, 1.2)};
                }}
                QPushButton:pressed {{
                    background-color: {self.adjust_color(color, 0.8)};
                }}
            """)

    def adjust_color(self, color, factor):
        from PyQt6.QtGui import QColor
        qcolor = QColor(color)
        return qcolor.lighter(int(100 * factor)).name()

    def update_image(self, cv_image, idx):
        self.cv_images[idx] = cv_image  # Przechowaj oryginalny obraz OpenCV
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_img = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(q_img)
        
        # Skaluj obraz z zachowaniem proporcji
        max_width = self.camera_labels[idx].width()
        max_height = self.camera_labels[idx].height()
        scaled_pixmap = pixmap.scaled(max_width, max_height, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
        
        self.camera_labels[idx].setPixmap(scaled_pixmap)

    def resizeEvent(self, event):
        # Aktualizuj obrazy po zmianie rozmiaru okna
        for idx in range(len(self.camera_labels)):
            if self.cv_images[idx] is not None:
                self.update_image(self.cv_images[idx], idx)
        super().resizeEvent(event)


class CameraWindow(QWidget):
    def __init__(self, camera_label, title):
        super().__init__()
        self.setWindowTitle(title)
        self.setGeometry(200, 200, 640, 480)
        self.setMinimumSize(200, 150)  # Ustaw minimalny rozmiar okna
        layout = QVBoxLayout()
        self.label = QLabel()
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Wyśrodkuj obraz
        self.label.setScaledContents(False)  # Wyłącz automatyczne skalowanie
        self.label.setStyleSheet("background-color: black;")  # Czarny kolor tła
        layout.addWidget(self.label)
        self.setLayout(layout)
        self.camera_label = camera_label
        self.update_image()

    def update_image(self):
        pixmap = self.camera_label.pixmap()
        if pixmap:
            # Skaluj obraz z zachowaniem proporcji
            scaled_pixmap = pixmap.scaled(self.label.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
            self.label.setPixmap(scaled_pixmap)
        QTimer.singleShot(30, self.update_image)

    def resizeEvent(self, event):
        # Aktualizuj obraz po zmianie rozmiaru okna
        self.update_image()
        super().resizeEvent(event)