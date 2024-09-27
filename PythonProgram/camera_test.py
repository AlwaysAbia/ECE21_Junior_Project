import cv2
from ultralytics import YOLO
import time

# Load the YOLO model
model = YOLO('BestFaces.pt', task='detect')

# Initialize the webcam
cap = cv2.VideoCapture(0)  # Use 0 for default camera, or change to the appropriate camera index

cap.set(cv2.CAP_PROP_FPS, 20)
print(cap.get(cv2.CAP_PROP_FPS))

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()
    if not ret:
        break  # Exit if frame is not read correctly

    # Run YOLOv8 inference on the frame
    results = model(frame)

    # Visualize the results on the frame
    annotated_frame = results[0].plot()

    # Display the annotated frame
    cv2.imshow("YOLOv8 Inference", annotated_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close all windows
cap.release()
cv2.destroyAllWindows()