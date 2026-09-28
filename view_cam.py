import cv2

# ==============================================================================
# ESP32-CAM Video Stream URL
# Replace with your ESP32-CAM IP (check Arduino Serial Monitor after booting)
# ==============================================================================
CAM_IP = "192.168.137.34"  # ESP32-CAM live stream IP
STREAM_URL = f"http://{CAM_IP}:81/stream"

print(f"Connecting to ESP32-CAM at: {STREAM_URL}")
print("Press 'q' or 'ESC' inside the video window to quit.")

cap = cv2.VideoCapture(STREAM_URL)

if not cap.isOpened():
    print(f"\n[Error] Could not open video stream from: {STREAM_URL}")
    print("Please check:")
    print("1. Is the ESP32-CAM powered on and connected to the same Wi-Fi?")
    print("2. Is the IP address correct?")
    exit(1)

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        print("[Warning] Failed to grab frame. Reconnecting...")
        continue

    # Display the live stream
    cv2.imshow("ESP32-CAM Live Stream", frame)

    # Press 'q' or ESC to exit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:
        break

cap.release()
cv2.destroyAllWindows()
print("Stream closed.")
