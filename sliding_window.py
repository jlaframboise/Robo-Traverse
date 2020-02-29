import cv2
import pandas as pd
import xml.etree.ElementTree as ET
import time

def sliding_window(image, stepSize, windowSize):
    for y in range(0, image.shape[0], stepSize):
        for x in range(0, image.shape[1], stepSize):
            yield (x, y, image[y:y + windowSize[1], x:x + windowSize[0]])

def overlap(l1, r1, l2, r2):
    if(l1[0] > r2[0] or l2[0] > r1[0]): 
        return False
    if(l1[1] > r2[1] or l2[1] > r1[1]):
        return False
    return True

image = cv2.imread('/Users/jessemaccormac/Desktop/Design Teams/QMIND/Scripts/02:20:20-0000.jpg')
(winW, winH) = (960, 360)

labels = []
xml = ET.parse('/Users/jessemaccormac/Desktop/Design Teams/QMIND/Scripts/02:20:20-0000.xml')
annot = xml.getroot()
fName = annot.find('filename').text
size = annot.find('size')
width = int(size.find('width').text)
height = int(size.find('height').text)
# if width<40 or height<40 or width >999999 or height> 999999:
#     continue

objects = annot.findall('object')
if len(objects)!=1:
    pass
elif objects[0].find('name').text=='pothole':
    box = objects[0].find('bndbox')
    
    pothole_x1 = int(box.find("xmin").text)
    pothole_y1 = int(box.find("ymin").text)
    pothole_x2 = int(box.find("xmax").text)
    pothole_y2 = int(box.find("ymax").text)
    
    # if xmin<0 or xmax>width or ymin<0 or ymax>height:
    #     continue
    
#     labels.append([fName, width, height, "pothole", xmin, ymin, xmax, ymax])
# labelDf = pd.DataFrame(labels, 
#                        columns=["filename", "width", 'height', "class", 'xmin', 'ymin', 'xmax', 'ymax'])

#Pothole bounding box
cv2.rectangle(image, (pothole_x1, pothole_y1), (pothole_x2, pothole_y2), (255, 0, 0), 2)

for (sliding_x, sliding_y, window) in sliding_window(image, stepSize=128, windowSize=(winW, winH)):
    if window.shape[0] != winH or window.shape[1] != winW:
        continue
    intersection_x1 = max(pothole_x1, sliding_x)
    intersection_y1 = max(pothole_y1, sliding_y)
    intersection_x2 = min(pothole_x2, sliding_x+winW)
    intersection_y2 = min(pothole_y2, sliding_y+winH)

    clone = image.copy()
    #Sliding window
    cv2.rectangle(clone, (sliding_x, sliding_y), (sliding_x + winW, sliding_y + winH), (0, 255, 0), 2)
    
    if (overlap((pothole_x1, pothole_y1), (pothole_x2, pothole_y2), (sliding_x, sliding_y), (sliding_x+winW, sliding_y+winH))):
        #Intersection box
        cv2.rectangle(clone, (intersection_x1, intersection_y1), (intersection_x2, intersection_y2), (0, 0, 255), 2)
        interArea = max(0, intersection_x2 - intersection_x1 + 1) * max(0, intersection_y2 - intersection_y1 + 1)

        potholeArea = (pothole_x2 - pothole_x1 + 1) * (pothole_y2 - pothole_y1 + 1)

        if(interArea > (potholeArea/3)):
            print("Pothole present")
    
    cv2.imshow("Window", clone)
    cv2.waitKey(1)
    time.sleep(0.025)
