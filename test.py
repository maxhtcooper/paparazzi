import numpy as np
import cv2
import os
import math
from itertools import combinations



class ObjectDetector():
    
    def __init__(self):
        super().__init__()
        
        self.model = cv2.dnn.readNet('yolov3.weights', 'yolov3.cfg')
        
        self.layer_name = self.model.getLayerNames()
        self.output_layers = [self.layer_name[i - 1] for i in self.model.getUnconnectedOutLayers()]

        self.NMS_THRESHOLD = 0.01
        self.MIN_CONFIDENCE = 0.05
        
    def yolo(self, image):
        (H, W) = image.shape[:2]
        
        # YOLOv3 full works best at 416x416 or 608x608
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.model.setInput(blob)
        layerOutputs = self.model.forward(self.output_layers)

        boxes = []
        centroids = []
        confidences = []

        for output in layerOutputs:
            for detection in output:
                # detection[4] is the 'Objectness' score - "Is there something here?"
                # detection[5:] are the scores for the 80 COCO classes
                
                scores = detection[5:]
                classID = np.argmax(scores)
                
                # Change: We take the MAX of (Objectness * Class Probability) 
                # to find the strongest match for ANY category.
                confidence = scores[classID] 

                # Lower this to 0.1 or 0.2 to see "unidentified" objects
                if confidence > self.MIN_CONFIDENCE:
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")

                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    boxes.append([x, y, int(width), int(height)])
                    centroids.append((centerX, centerY))
                    confidences.append(float(confidence))

        # NMS is crucial for full YOLOv3 as it predicts many overlapping boxes
        idzs = cv2.dnn.NMSBoxes(boxes, confidences, self.MIN_CONFIDENCE, self.NMS_THRESHOLD)
        
        results = []
        if len(idzs) > 0:
            for i in idzs.flatten():
                # (Confidence, (x, y, x2, y2), (cx, cy))
                res = (confidences[i], (boxes[i][0], boxes[i][1], boxes[i][0]+boxes[i][2], boxes[i][1]+boxes[i][3]), centroids[i])
                results.append(res)
                
        return results, self.calculate_max_dist(centroids)

    def calculate_max_dist(self, centroids):
        max_dist = 0.0
        if len(centroids) >= 2:
            for c1, c2 in combinations(centroids, 2):
                dist = math.hypot(c1[0] - c2[0], c1[1] - c2[1])
                if dist > max_dist:
                    max_dist = dist
        return max_dist


object_detector = ObjectDetector()

image_path = 'AE4317_2019_datasets/cyberzoo_poles/20190121-135009/106044555.jpg'

image = cv2.imread(image_path)

if image is not None:
    results, dist = object_detector.yolo(image)
    
    # Draw Results
    for (conf, box, centroid) in results:
        # Draw Bounding Box (Green)
        cv2.rectangle(image, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
        # Draw Centroid (Red dot)
        cv2.circle(image, centroid, 5, (0, 0, 255), -1)

    # Output
    cv2.imshow("Detection Output", image)
    cv2.waitKey(0) # Press any key to close
    cv2.destroyAllWindows()
else:
    print("Failed to load image. Double check the path!")