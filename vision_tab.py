from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QTabWidget
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt6.QtGui import QPixmap, QImage, QColor
import cv2  # Dodaj ten import

class ImageProcessingThread(QThread):
    image_processed = pyqtSignal(object, int)  # Sygnał do przekazywania przetworzonego obrazu

    def __init__(self, cv_image, idx):
        super().__init__()
        self.cv_image = cv_image
        self.idx = idx

    def run(self):
        # Przetwarzanie obrazu w osobnym wątku
        if self.cv_image is not None:
            # Zmniejszenie rozmiaru obrazu
            scale_percent = 50  # Przykładowe zmniejszenie rozmiaru o 50%
            width = int(self.cv_image.shape[1] * scale_percent / 100)
            height = int(self.cv_image.shape[0] * scale_percent / 100)
            dim = (width, height)
            resized_image = cv2.resize(self.cv_image, dim, interpolation=cv2.INTER_AREA)

            # Konwersja do QImage
            height, width, channel = resized_image.shape
            bytes_per_line = 3 * width
            q_img = QImage(resized_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888).rgbSwapped()
            self.image_processed.emit(q_img, self.idx)  # Emitowanie przetworzonego obrazu


class VisionTab(QWidget):
    def __init__(self, open_camera_window_callback):
        super().__init__()
        self.open_camera_window_callback = open_camera_window_callback
        self.init_ui()
        self.cv_images = [None] * 4  # Lista do przechowywania oryginalnych obrazów OpenCV
        self.threads = [None] * 4  # Lista do przechowywania wątków przetwarzania obrazów

    def init_ui(self):
        layout = QVBoxLayout()
        self.vision_tabs = QTabWidget()
        self.camera_labels = []
        for i in range(4):
            tab = QWidget()
            tab_layout = QVBoxLayout()
            label = QLabel(f'Oczekiwanie na obraz z kamery {i + 1}...')
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setStyleSheet("background-color: black; color: white;")
            label.setScaledContents(False)
            label.setMinimumHeight(200)
            self.camera_labels.append(label)
            open_button = QPushButton(f'Otwórz kamerę {i + 1}')
            open_button.clicked.connect(lambda _, idx=i: self.open_camera_window_callback(idx))
            self.style_button(open_button, '#3498DB')
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
        # Uruchom wątek do przetwarzania obrazu
        if self.threads[idx] is None or not self.threads[idx].isRunning():
            self.threads[idx] = ImageProcessingThread(cv_image, idx)
            self.threads[idx].image_processed.connect(self.on_image_processed)
            self.threads[idx].start()

    def on_image_processed(self, q_img, idx):
        # Aktualizacja etykiety obrazem
        pixmap = QPixmap.fromImage(q_img)
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
        QTimer.singleShot(30, self.update_image)  # Aktualizuj obraz co 30 ms

    def resizeEvent(self, event):
        # Aktualizuj obraz po zmianie rozmiaru okna
        self.update_image()
        super().resizeEvent(event)