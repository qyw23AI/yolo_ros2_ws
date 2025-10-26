from ultralytics import YOLO 

# 加载预训练模型
model = YOLO('yolov8n.pt')
#定义训练参数
data = 'yolo-bvn.yaml'
workers = 8
epoches = 100
batch = 16
model.train(data=data, epochs=epoches, batch=batch, workers=workers)