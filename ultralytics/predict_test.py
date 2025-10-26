from ultralytics import YOLO
yolo = YOLO("./yolov8n.pt",task = "classify")
results = yolo(source="./ultralytics/assets/bus.jpg", save=True, conf = 0.1)