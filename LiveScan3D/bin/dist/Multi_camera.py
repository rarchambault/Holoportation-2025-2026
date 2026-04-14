import sys
import cv2
import numpy as np
import glob
import os
import re
import time
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QLabel, QFrame)
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QImage, QPixmap

# ==========================================
# CONFIGURATION
# ==========================================

MAX_INDEX = 5              
MAX_RGB_CAMERAS = 3         
FRAME_WIDTH = 640           
FRAME_HEIGHT = 360          

MIN_BRIGHTNESS = 10
MIN_STD = 5
MIN_COLOR_STD = 8

FACES_FOLDER = "faces"      
REFERENCE_DISTANCE_CM = 30.0
TARGET_SIZE = 140           

# Detection Limits (Size)
MIN_CONTOUR_AREA = 1200     
MAX_CONTOUR_AREA = 80000    
REF_MIN_CONTOUR_AREA = 400  

# Stricter matching to prevent false positives
MATCH_THRESHOLD = 0.55      
MAX_MARKERS_PER_FRAME = 4

# Distance multiplier
DISTANCE_MULTIPLIER = 0.25 

# CALIBRATION CRITERIA
GOOD_BRIGHTNESS_RANGE = (40, 220)  
GOOD_DISTANCE_RANGE = (30, 150)    

# ==========================================
# PART 1: HELPER FUNCTIONS
# ==========================================

def order_points(pts):
    pts = np.array(pts, dtype="float32")
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(diff)]
    bl = pts[np.argmax(diff)]
    return np.array([tl, tr, br, bl], dtype="float32")

def detect_markers_live(img_bgr):
    markers = []
    if img_bgr is None: return markers

    h_img, w_img = img_bgr.shape[:2]
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:MAX_MARKERS_PER_FRAME]

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_CONTOUR_AREA or area > MAX_CONTOUR_AREA: 
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        if w == 0 or h == 0: continue

        cx, cy = x + w / 2.0, y + h / 2.0
        
        if not (0.25 * w_img <= cx <= 0.75 * w_img and 0.25 * h_img <= cy <= 0.75 * h_img): 
            continue

        aspect = min(w, h) / max(w, h)
        if aspect < 0.25: continue

        quad = np.array([[x, y], [x+w, y], [x+w, y+h], [x, y+h]], dtype="float32")
        ordered = order_points(quad)
        dst_pts = np.array([[0, 0], [TARGET_SIZE - 1, 0], [TARGET_SIZE - 1, TARGET_SIZE - 1], [0, TARGET_SIZE - 1]], dtype="float32")

        M = cv2.getPerspectiveTransform(ordered, dst_pts)
        warp = cv2.warpPerspective(img_bgr, M, (TARGET_SIZE, TARGET_SIZE))

        warp_gray = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)
        _, warp_bin = cv2.threshold(warp_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        h1 = np.linalg.norm(ordered[0] - ordered[3])
        h2 = np.linalg.norm(ordered[1] - ordered[2])
        markers.append((warp_bin, ordered.reshape(-1, 1, 2).astype(np.int32), (h1 + h2) / 2.0))

    return markers

def extract_marker_reference(img_bgr):
    if img_bgr is None: return None, None
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_cnt, best_area = None, 0.0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < REF_MIN_CONTOUR_AREA: continue
        x, y, w, h = cv2.boundingRect(cnt)
        if min(w,h)/max(w,h) < 0.25: continue
        if area > best_area:
            best_area, best_cnt = area, cnt

    if best_cnt is None: return None, None

    x, y, w, h = cv2.boundingRect(best_cnt)
    quad = np.array([[x, y], [x+w, y], [x+w, y+h], [x, y+h]], dtype="float32")
    ordered = order_points(quad)
    dst_pts = np.array([[0, 0], [TARGET_SIZE-1, 0], [TARGET_SIZE-1, TARGET_SIZE-1], [0, TARGET_SIZE-1]], dtype="float32")
    
    M = cv2.getPerspectiveTransform(ordered, dst_pts)
    warp = cv2.warpPerspective(img_bgr, M, (TARGET_SIZE, TARGET_SIZE))
    warp_gray = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)
    _, warp_bin = cv2.threshold(warp_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    h1 = np.linalg.norm(ordered[0] - ordered[3])
    h2 = np.linalg.norm(ordered[1] - ordered[2])
    return warp_bin, (h1+h2)/2.0

def load_reference_markers():
    patterns = []
    for ext in ("*.jpg", "*.jpeg", "*.png", "*.bmp"):
        patterns.extend(glob.glob(os.path.join(FACES_FOLDER, ext)))
    
    face_paths = sorted([p for p in patterns if os.path.basename(p).lower().startswith("face")])
    if not face_paths: return {}, None

    raw_templates = {}
    ref_height_px = None

    for path in face_paths:
        fname = os.path.basename(path).lower()
        m = re.match(r"(face\d+)", fname)
        if not m: continue
        label = m.group(1)

        img = cv2.imread(path)
        if img is None: continue
        
        warp_bin, height_px = extract_marker_reference(img)
        if warp_bin is None: continue

        raw_templates.setdefault(label, []).append(warp_bin)
        if ref_height_px is None: ref_height_px = height_px

    if not raw_templates: return {}, None

    final_templates = {}
    for label, imgs in raw_templates.items():
        stack = np.stack(imgs, axis=0).astype(np.float32) / 255.0
        mean = np.mean(stack, axis=0)
        final_templates[label] = np.where(mean > 0.5, 255, 0).astype(np.uint8)
    
    return final_templates, ref_height_px

def similarity_score_multi_rot(test_img, ref_img):
    best = -1.0
    curr = test_img.copy()
    for _ in range(4):
        res = cv2.matchTemplate(curr.astype(np.float32), ref_img.astype(np.float32), cv2.TM_CCOEFF_NORMED)
        if float(res[0][0]) > best: best = float(res[0][0])
        curr = cv2.rotate(curr, cv2.ROTATE_90_CLOCKWISE)
    return best

def is_rgb_camera(cap, test_frames=5):
    for _ in range(test_frames):
        ret, frame = cap.read()
        if not ret or frame is None: continue
        if frame.ndim != 3 or frame.shape[2] != 3: continue
        if frame.mean() <= MIN_BRIGHTNESS or frame.std() <= MIN_STD: continue
        if frame.astype(np.float32).std(axis=2).mean() > MIN_COLOR_STD: return True
    return False

def scan_rgb_cameras(max_index=MAX_INDEX, max_rgb=MAX_RGB_CAMERAS):
    rgb_cams = []
    for i in range(max_index):
        if len(rgb_cams) >= max_rgb: break
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        if is_rgb_camera(cap):
            rgb_cams.append((i, cap))
        else:
            cap.release()
    return rgb_cams

# ==========================================
# PART 2: MULTI-CAMERA BACKGROUND THREAD
# ==========================================

class VisionThread(QThread):
    frame_ready = Signal(int, QImage)
    status_msg = Signal(str)
    stats_ready = Signal(int, dict) 
    cameras_detected = Signal(int) # NEW: Tells UI how many cameras are active

    def __init__(self):
        super().__init__()
        self.running = True
        self.last_times = {}

    def run(self):
        self.status_msg.emit("Loading templates...")
        templates, ref_h = load_reference_markers()
        if not templates or ref_h is None:
            self.status_msg.emit("ERROR: Missing/invalid 'faces' folder.")
            self.cameras_detected.emit(0)
            return
            
        k_factor = REFERENCE_DISTANCE_CM * ref_h
        
        self.status_msg.emit("Scanning cameras...")
        cams = scan_rgb_cameras()
        
        # Emit the exact number of active cameras found
        active_count = len(cams)
        self.cameras_detected.emit(active_count)
        
        if not cams:
            self.status_msg.emit("ERROR: No RGB cameras found.")
            return

        self.status_msg.emit(f"System Active. ({active_count} cameras)")
        
        for cam_id, _ in cams:
            self.last_times[cam_id] = time.time()

        while self.running:
            for idx, (cam_id, cap) in enumerate(cams):
                ret, frame = cap.read()
                if not ret: continue

                h, w = frame.shape[:2]

                pt1 = (int(w * 0.25), int(h * 0.25))
                pt2 = (int(w * 0.75), int(h * 0.75))
                cv2.rectangle(frame, pt1, pt2, (40, 40, 40), 1)

                current_time = time.time()
                time_diff = current_time - self.last_times[cam_id]
                fps = 1.0 / time_diff if time_diff > 0 else 0
                self.last_times[cam_id] = current_time

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                avg_brightness = gray.mean()
                clarity = cv2.Laplacian(gray, cv2.CV_64F).var()

                markers = detect_markers_live(frame)
                closest_dist = -1.0
                detected_faces = 0
                
                for marker_warp, quad, marker_height_px in markers:
                    if marker_height_px <= 0: continue
                    best_label, best_score = "Unknown", -1.0
                    for label, tmpl in templates.items():
                        score = similarity_score_multi_rot(marker_warp, tmpl)
                        if score > best_score:
                            best_score, best_label = score, label
                    
                    if best_score < MATCH_THRESHOLD: best_label = "Unknown"
                    
                    # IGNORE FALSE POSITIVES
                    if best_label == "Unknown": continue 
                    
                    detected_faces += 1
                    dist = (k_factor / marker_height_px) * DISTANCE_MULTIPLIER
                    
                    if closest_dist < 0 or dist < closest_dist:
                        closest_dist = dist

                    M = cv2.moments(quad)
                    cx, cy = int(M["m10"]/M["m00"]) if M["m00"] != 0 else quad[0,0][0], int(M["m01"]/M["m00"]) if M["m00"] != 0 else quad[0,0][1]
                    
                    if dist < GOOD_DISTANCE_RANGE[0]:
                        color = (0, 0, 255)
                        cv2.drawContours(frame, [quad], -1, color, 3)
                        cv2.putText(frame, "TOO CLOSE! Move Back", (cx - 80, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    elif dist > GOOD_DISTANCE_RANGE[1]:
                        color = (0, 165, 255) # Orange
                        cv2.drawContours(frame, [quad], -1, color, 2)
                        cv2.putText(frame, "TOO FAR!", (cx - 40, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    else:
                        color = (0, 255, 0)
                        cv2.drawContours(frame, [quad], -1, color, 2)
                        cv2.putText(frame, f"{dist:.0f}cm", (cx - 20, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

                stats = {
                    "fps": fps,
                    "res": f"{w}x{h}",
                    "faces": detected_faces,
                    "brightness": avg_brightness,
                    "clarity": clarity,
                    "dist": closest_dist
                }
                self.stats_ready.emit(idx + 1, stats)

                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).copy()
                q_img = QImage(rgb_frame.data, w, h, 3 * w, QImage.Format_RGB888)
                self.frame_ready.emit(idx + 1, q_img)

    def stop(self):
        self.running = False
        self.wait()

# ==========================================
# PART 3: PURE INFORMATION UI
# ==========================================

class DashboardApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Camera Diagnostics Dashboard")
        self.resize(1200, 600)
        self.setStyleSheet("background-color: #0d1117; color: #c9d1d9; font-family: 'Segoe UI', sans-serif;")
        
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        top_layout = QHBoxLayout()
        
        self.status_label = QLabel("Initializing System...")
        self.status_label.setStyleSheet("font-weight: bold; color: #58a6ff; font-size: 14px;")
        
        self.ready_banner = QLabel("NOT READY: Waiting for cameras")
        self.ready_banner.setStyleSheet("background-color: #3b0000; color: #ff6666; font-weight: bold; font-size: 16px; padding: 10px; border-radius: 5px;")
        self.ready_banner.setAlignment(Qt.AlignCenter)
        
        top_layout.addWidget(self.status_label, stretch=1)
        top_layout.addWidget(self.ready_banner, stretch=2)
        layout.addLayout(top_layout)
        
        # Dynamic tracking
        self.active_camera_count = 0
        self.camera_ready_states = {} 
        
        video_layout = QHBoxLayout()
        self.feeds = {}
        self.stats = {}
        
        for i in range(1, 4):
            container = QFrame()
            container.setStyleSheet("background-color: #161b22; border: 1px solid #30363d; border-radius: 8px;")
            vbox = QVBoxLayout(container)
            vbox.setContentsMargins(10, 10, 10, 10)
            
            title = QLabel(f"CAMERA {i}")
            title.setStyleSheet("font-weight: bold; color: #8b949e; border: none;")
            title.setAlignment(Qt.AlignCenter)
            vbox.addWidget(title)

            lbl = QLabel("NO SIGNAL")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("background-color: #000000; border: 1px solid #000; border-radius: 4px;")
            lbl.setMinimumSize(320, 240)
            self.feeds[i] = lbl
            vbox.addWidget(lbl, stretch=1)
            
            stats_lbl = QLabel("Waiting for data...")
            stats_lbl.setStyleSheet("""
                color: #e6edf3; 
                font-family: monospace; 
                font-size: 14px; 
                background: #0d1117; 
                padding: 10px; 
                border-radius: 4px;
                border: 1px solid #21262d;
            """)
            stats_lbl.setAlignment(Qt.AlignCenter)
            self.stats[i] = stats_lbl
            vbox.addWidget(stats_lbl)
            
            video_layout.addWidget(container)
            
        layout.addLayout(video_layout, stretch=1)

        self.vision_thread = VisionThread()
        self.vision_thread.frame_ready.connect(self.update_feed)
        self.vision_thread.status_msg.connect(self.status_label.setText)
        self.vision_thread.stats_ready.connect(self.update_stats)
        self.vision_thread.cameras_detected.connect(self.set_camera_count)
        
        self.vision_thread.start()

    def set_camera_count(self, count):
        self.active_camera_count = count

    def update_feed(self, cam_idx, q_img):
        if cam_idx in self.feeds:
            pixmap = QPixmap.fromImage(q_img)
            scaled = pixmap.scaled(self.feeds[cam_idx].size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.feeds[cam_idx].setPixmap(scaled)
            
    def update_stats(self, cam_idx, stats):
        if cam_idx in self.stats:
            
            faces_ok = stats['faces'] > 0
            bright_ok = GOOD_BRIGHTNESS_RANGE[0] <= stats['brightness'] <= GOOD_BRIGHTNESS_RANGE[1]
            dist_ok = GOOD_DISTANCE_RANGE[0] <= stats['dist'] <= GOOD_DISTANCE_RANGE[1] and stats['dist'] > 0
            
            # Update dynamic dictionary
            self.camera_ready_states[cam_idx] = faces_ok and bright_ok and dist_ok
            
            # Check readiness based ONLY on connected cameras
            if self.active_camera_count > 0 and len(self.camera_ready_states) == self.active_camera_count:
                if all(self.camera_ready_states.values()):
                    self.ready_banner.setText("🌟 READY FOR CALIBRATION 🌟")
                    self.ready_banner.setStyleSheet("background-color: #004d00; color: #00ff00; font-weight: bold; font-size: 20px; padding: 10px; border-radius: 5px;")
                else:
                    self.ready_banner.setText("NOT READY: Check individual camera metrics")
                    self.ready_banner.setStyleSheet("background-color: #3b0000; color: #ff6666; font-weight: bold; font-size: 16px; padding: 10px; border-radius: 5px;")

            f_icon = "✅" if faces_ok else "❌"
            b_icon = "✅" if bright_ok else "❌"
            d_icon = "✅" if dist_ok else "❌"
            
            dist_val = f"{stats['dist']:.0f}cm" if stats['dist'] > 0 else "--"
            
            line1 = f"FPS: {stats['fps']:.1f}  |  Res: {stats['res']}  |  Faces: {stats['faces']} {f_icon}"
            line2 = f"Bright: {stats['brightness']:.0f} {b_icon} | Clarity: {stats['clarity']:.0f} | Dist: {dist_val} {d_icon}"
            
            self.stats[cam_idx].setText(f"{line1}\n{line2}")

    def closeEvent(self, event):
        self.vision_thread.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DashboardApp()
    window.show()
    sys.exit(app.exec())