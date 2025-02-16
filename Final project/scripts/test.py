#! /usr/bin/python3

import os
import rospy
import torch
import cv2
import numpy as np
import time
from PIL import Image
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from torchvision import models, transforms

# Initial setup
num_classes = 3
model_path = '/home/jovyan/catkin_ws/src/final/model.pth'

model_color = models.resnet18(pretrained=False)
num_ftrs = model_color.fc.in_features
model_color.fc = torch.nn.Linear(num_ftrs, num_classes)
model_color.load_state_dict(torch.load(model_path))
model_color.eval()

model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/jovyan/catkin_ws/src/final/best.pt')
classes = ['traffic light', 'speedlimit', 'crosswalk', 'stop']
bridge = CvBridge()

# ROS setup
pub = rospy.Publisher('detected_classes', String, queue_size=10)

# Color detection function
def detect_color(image):
    # 圖片處理
    image = image.convert('RGB')
    preprocess = transforms.Compose([
        transforms.Resize(256),
        transforms.CenterCrop(224),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])

    input_tensor = preprocess(image)
    input_batch = input_tensor.unsqueeze(0)  # 創建一個假的 batch 維度以符合模型的輸入需求

    # 模型預測
    with torch.no_grad():
        output = model_color(input_batch)

    # 得到預測結果
    _, predicted = torch.max(output, 1)

    classes = ['green', 'red', 'yellow']
    color = classes[predicted.item()]
    return color

# Image detection function
def detect_image_and_publish(cv2_img):
    if cv2_img is None:
        return
    cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(cv2_img)
    # 偵測
    results = model(pil_img)

    # 取得偵測結果的類別ID
    class_ids = results.xyxy[0][:, 5].int().tolist()

    # 將類別ID轉換為類別名稱
    class_names = [classes[i] for i in class_ids]
    
    if 'traffic light' in class_names:
        # 提取交通燈的邊界框
        traffic_light_boxes = results.xyxy[0][results.xyxy[0][:, 5].int() == classes.index('traffic light')]

        # 對每一個紅綠燈進行顏色判斷
        for box in traffic_light_boxes:
            # 提取紅綠燈的區域
            light = pil_img.crop(box[:4].tolist())

            color = detect_color(light)
            
            print(f'Detected color for traffic light: {color}')

        pub.publish(String(color))
    else:
        print(f'Detected classes: {class_names}')
        pub.publish(String(class_names[0]))
    
    time.sleep(0.5)  # 每次偵測後等待 0.5 秒

# Image callback function
last_time = time.time()
def image_callback(msg):
    global last_time
    # 如果距離上次偵測的時間少於 0.5秒，則跳過此次偵測
    if time.time() - last_time < 0.5:
        return
    try:
        cv2_img = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        detect_image_and_publish(cv2_img)
        last_time = time.time()  # 更新最後偵測的時間
    except CvBridgeError as e:
        print(e)

# Main function
def main():
    rospy.init_node('image_listener')
    image_topic = "/camera/rgb/image_raw/compressed"
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
