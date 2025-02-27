from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QTabWidget, QHBoxLayout
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt6.QtGui import QPixmap, QImage, QColor
import cv2


class ImageProcessingThread(QThread):
    image_processed = pyqtSignal(object, int)

    def __init__(self, cv_image, idx):
        super().__init__()
        self.cv_image = cv_image
        self.idx = idx

    def run(self):
        if self.cv_image is not None:
            scale_percent = 50
            width = int(self.cv_image.shape[1] * scale_percent / 100)
            height = int(self.cv_image.shape[0] * scale_percent / 100)
            dim = (width, height)
            resized_image = cv2.resize(self.cv_image, dim, interpolation=cv2.INTER_AREA)

            height, width, channel = resized_image.shape
            bytes_per_line = 3 * width
            q_img = QImage(resized_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888).rgbSwapped()
            self.image_processed.emit(q_img, self.idx)

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

class VisionTab(QWidget):
    def __init__(self, open_camera_window_callback):
        super().__init__()
        self.open_camera_window_callback = open_camera_window_callback
        self.init_ui()
        self.cv_images = [None] * 4  # Lista do przechowywania oryginalnych obrazów OpenCV
        self.threads = [None] * 4  # Lista do przechowywania wątków przetwarzania obrazów
        self.subscriptions = [True] * 4  # Lista do śledzenia, czy dana kamera jest włączona

    def init_ui(self):
        layout = QVBoxLayout()
        self.vision_tabs = QTabWidget()
        self.camera_labels = []
        self.toggle_buttons = []  # Lista przycisków do przełączania subskrypcji

        for i in range(4):
            tab = QWidget()
            tab_layout = QVBoxLayout()

            label = QLabel(f'Oczekiwanie na obraz z kamery {i + 1}...')
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setStyleSheet("background-color: black; color: white;")
            label.setScaledContents(False)
            label.setMinimumHeight(200)

            open_button = QPushButton(f'Otwórz kamerę {i + 1}')
            open_button.clicked.connect(lambda _, idx=i: self.open_camera_window_callback(idx))
            self.style_button(open_button, '#3498DB')

            toggle_button = QPushButton("Wyłącz subskrypcję")  # Domyślnie subskrypcja jest włączona
            toggle_button.setCheckable(True)
            toggle_button.setChecked(True)
            # Zmieniamy sposób przypisania funkcji do kliknięcia przycisku
            toggle_button.clicked.connect(self.create_subscription_handler(i, toggle_button))
            self.style_button(toggle_button, '#E74C3C')

            self.camera_labels.append(label)
            self.toggle_buttons.append(toggle_button)

            tab_layout.addWidget(label)
            tab_layout.addWidget(open_button)
            tab_layout.addWidget(toggle_button)
            tab.setLayout(tab_layout)
            self.vision_tabs.addTab(tab, f'Kamera {i + 1}')

        layout.addWidget(self.vision_tabs)
        self.setLayout(layout)
    
    def style_button(self, button, color):
        button.setStyleSheet(f"background-color: {color}; color: white; font-size: 14px; border-radius: 5px; padding: 5px;")

    def create_subscription_handler(self, idx, toggle_button):
        """ Tworzy funkcję do obsługi kliknięcia przycisku subskrypcji dla danej kamery """
        def handler():
            self.toggle_subscription(idx, toggle_button)
        return handler

    def toggle_subscription(self, idx, button):
        """ Włącza lub wyłącza subskrypcję kamery """
        self.subscriptions[idx] = not self.subscriptions[idx]

        if self.subscriptions[idx]:
            button.setText("Wyłącz subskrypcję")
            button.setStyleSheet("background-color: #E74C3C; color: white;")
        else:
            button.setText("Włącz subskrypcję")
            button.setStyleSheet("background-color: #2ECC71; color: white;")

    def update_image(self, cv_image, idx):
        """ Aktualizuje obraz tylko dla włączonych kamer """
        if not self.subscriptions[idx]:  # Jeśli subskrypcja jest wyłączona, ignorujemy obraz
            self.camera_labels[idx].clear()
            return

        if self.threads[idx] is None or not self.threads[idx].isRunning():
            self.threads[idx] = ImageProcessingThread(cv_image, idx)
            self.threads[idx].image_processed.connect(self.on_image_processed)
            self.threads[idx].start()

    def on_image_processed(self, q_img, idx):
        """ Aktualizuje etykietę obrazem tylko jeśli subskrypcja jest włączona """
        if not self.subscriptions[idx]:  
            return

        pixmap = QPixmap.fromImage(q_img)
        scaled_pixmap = pixmap.scaled(self.camera_labels[idx].width(),
                                      self.camera_labels[idx].height(),
                                      Qt.AspectRatioMode.KeepAspectRatio,
                                      Qt.TransformationMode.SmoothTransformation)
        self.camera_labels[idx].setPixmap(scaled_pixmap)
