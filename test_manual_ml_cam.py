"""
Manual ML Trash Detection Tester with ESP32-CAM Stream
======================================================
Runs trained YOLOv8 model on the live ESP32-CAM stream for manual testing.
No autonomous movement - allows you to verify ML detections safely.

Features:
  - Live real-time YOLOv8m detection on NVIDIA RTX 4060 GPU
  - Shows bounding boxes, class labels, and confidence %
  - Displays which sorting bin each item belongs to (Bin 1, 2, or 3)
  - Interactive controls:
      [+] / [-] : Increase / Decrease confidence threshold
      [P]       : Manually send test PICK command to ESP32 DevKit
      [S]       : Save annotated snapshot photo
      [Q] / ESC : Exit
"""

import os
import sys
import time
import socket
from pathlib import Path
import cv2
import torch
from ultralytics import YOLO

ROOT_DIR = Path(__file__).resolve().parent

# ==============================================================================
# CONFIGURATION
# ==============================================================================
ESP32_CAM_URL = "http://192.168.137.34:81/stream"  # ESP32-CAM Stream URL
DEVKIT_IP     = "192.168.137.218"                  # ESP32 DevKit IP
DEVKIT_PORT   = 8888

# Model weights
WEIGHTS_PATH = str(ROOT_DIR / "runs" / "detect" / "trash_yolov8m_ultra_acc" / "weights" / "best.pt")
if not os.path.exists(WEIGHTS_PATH):
    WEIGHTS_PATH = str(ROOT_DIR / "runs" / "detect" / "trash_finetune" / "weights" / "best.pt")
if not os.path.exists(WEIGHTS_PATH):
    WEIGHTS_PATH = "yolov8m.pt"

# Waste Sorting Bin Categories
def get_sorting_bin(cls_name: str) -> tuple:
    name_upper = cls_name.upper()
    if "GLASS" in name_upper:
        return "Bin 3 (Center - 0 deg)", "Glass / General", (255, 165, 0)
    elif "PLASTIC" in name_upper or "BOTTLE" in name_upper:
        return "Bin 1 (Left - 45 deg)", "Plastics", (0, 255, 255)
    elif "CAN" in name_upper or "BATTERY" in name_upper or "POP TAB" in name_upper:
        return "Bin 2 (Right - 135 deg)", "Metals", (0, 165, 255)
    else:
        return "Bin 3 (Center - 0 deg)", "General Recyclable", (200, 200, 200)

def probe_stream(url: str, timeout: float = 2.0) -> bool:
    """Fast check if camera HTTP stream is actively serving frames."""
    try:
        import urllib.request
        req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
        res = urllib.request.urlopen(req, timeout=timeout)
        return res.status == 200
    except Exception:
        return False

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Manual ML Trash Detection Tester")
    parser.add_argument("--source", type=str, default=None, help="ESP32-CAM stream URL or '0' for laptop webcam")
    parser.add_argument("--conf", type=float, default=0.40, help="Confidence threshold")
    args = parser.parse_args()

    # Determine video source
    source = args.source
    if source is None:
        source = ESP32_CAM_URL

    print("=" * 65)
    print(" Manual ML Trash Detection Tester - AI Vision")
    print("=" * 65)
    print(f" Requested Source : {source}")
    print(f" Model Weights    : {WEIGHTS_PATH}")
    print("=" * 65)

    # Initialize UDP socket for manual pick testing
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Load YOLOv8 Model on GPU
    device = "0" if torch.cuda.is_available() else "cpu"
    print(f"\n[AI] Loading YOLOv8 on {device.upper()}...")
    model = YOLO(WEIGHTS_PATH)

    # Connect to ESP32-CAM stream ONLY (No webcam fallback)
    print(f"\n[Vision] Connecting to ESP32-CAM stream: {source}...")
    cap = cv2.VideoCapture(source)
    retry_count = 0
    while not cap.isOpened() and retry_count < 5:
        retry_count += 1
        print(f" -> Connecting to ESP32-CAM... (Attempt {retry_count}/5)")
        time.sleep(1.0)
        cap = cv2.VideoCapture(source)

    if not cap.isOpened():
        print(f"\n[Error] Could not connect to ESP32-CAM stream at: {source}")
        print("Troubleshooting:")
        print(" 1. Ensure the ESP32-CAM is powered with a stable 5V supply.")
        print(" 2. Make sure GPIO 0 is DISCONNECTED from GND, and press RST.")
        print(" 3. Upload the updated esp32_cam_firmware.ino with PSRAM Enabled.")
        return

    print("\n" + "=" * 55)
    print(" Live ML Testing Active!")
    print("=" * 55)
    print(" Controls:")
    print("   [+] / [=]   : Increase confidence threshold (+0.05)")
    print("   [-] / [_]   : Decrease confidence threshold (-0.05)")
    print("   [P]         : Test sending ARM:PICK command to DevKit")
    print("   [S]         : Save snapshot to disk")
    print("   [Q] / [ESC] : Quit")
    print("=" * 55 + "\n")

    conf_thresh = 0.40
    prev_time = time.time()
    fps = 0.0
    last_detected_class = None

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.05)
                continue

            frame = cv2.resize(frame, (640, 480))

            # FPS calculation
            curr_time = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(curr_time - prev_time, 1e-5))
            prev_time = curr_time

            # Run YOLOv8 ML Inference
            infer_start = time.time()
            results = model.predict(frame, conf=conf_thresh, device=device, verbose=False)
            infer_ms = (time.time() - infer_start) * 1000

            annotated_frame = results[0].plot()
            boxes = results[0].boxes

            detected_summary = []
            last_detected_class = None

            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]
                conf = float(box.conf[0])
                bin_slot, bin_type, color = get_sorting_bin(cls_name)
                detected_summary.append(f"{cls_name} ({conf*100:.0f}%) -> {bin_slot}")
                last_detected_class = cls_name

            # Draw HUD
            hud_bg = annotated_frame.copy()
            cv2.rectangle(hud_bg, (0, 0), (640, 45), (20, 20, 20), -1)
            cv2.addWeighted(hud_bg, 0.7, annotated_frame, 0.3, 0, annotated_frame)

            top_info = f"FPS: {fps:.1f} | Latency: {infer_ms:.1f}ms | Conf: {conf_thresh:.2f} | Detected: {len(boxes)}"
            cv2.putText(annotated_frame, top_info, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Draw sorting bin destination overlay
            if detected_summary:
                for i, text in enumerate(detected_summary[:3]):
                    y_pos = 70 + (i * 25)
                    cv2.putText(annotated_frame, text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                    cv2.putText(annotated_frame, text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow("Manual ML Model Testing - ESP32-CAM", annotated_frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:
                break
            elif key == ord('+') or key == ord('='):
                conf_thresh = min(0.95, round(conf_thresh + 0.05, 2))
                print(f"[Controls] Confidence threshold: {conf_thresh:.2f}")
            elif key == ord('-') or key == ord('_'):
                conf_thresh = max(0.10, round(conf_thresh - 0.05, 2))
                print(f"[Controls] Confidence threshold: {conf_thresh:.2f}")
            elif key == ord('s') or key == ord('S'):
                snap_file = f"ml_test_snapshot_{int(time.time())}.jpg"
                cv2.imwrite(snap_file, annotated_frame)
                print(f"[Snapshot] Saved to {snap_file}")
            elif key == ord('p') or key == ord('P'):
                if last_detected_class:
                    cmd = f"ARM:PICK:{last_detected_class.upper()}"
                    udp_sock.sendto(cmd.encode('utf-8'), (DEVKIT_IP, DEVKIT_PORT))
                    print(f"📡 [Manual Pick Sent to DevKit] --> {cmd}")
                else:
                    print("⚠️ No waste object currently detected to pick.")

    finally:
        cap.release()
        udp_sock.close()
        cv2.destroyAllWindows()
        print("\n[Finished] Manual ML testing closed.")

if __name__ == "__main__":
    main()
