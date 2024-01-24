from ultralytics import YOLO
import os
import csv
import matplotlib.pylab as plt
import matplotlib.patches as patches
import matplotlib.image as mpimg

class_id  = None

model_path = os.path.join('.', 'runs', 'detect', 'train', 'weights', 'last.pt')

model = YOLO(model_path) # load trained moel

test_path = "/home/csmith/Desktop/School/Winter 2024/Winter Project/ws/Winter-Project/yolo_model/tello_arrow_model/code - tello/data/images/test"
# file = open("results.csv", 'w')

## get all files in the test files:
# for image_file in os.listdir(test_path):
image_path = os.path.join(test_path, 'right_48.JPG')

results = model.predict(image_path)


for result in results[0].boxes.data.tolist():
    x1, y1, x2, y2, score, class_id = result
    print(f"X1 = {x1}")
    print(f"Y1 = {y1}")
    print(f"X2 = {x2}")
    print(f"Y2 = {y2}")
    # image_num = int(image_file[:-4])

    # writer.writerow([str(image_num), str(class_id), str(x1), str(y1), str(x2), str(y2), str(score)])
    # row = str(image_num)+', '+str(class_id)+', '+str(x1)+', '+str(y1)+', '+str(x2)+', '+str(y2)+', '+str(score)+'\n'
    # file.write(row)

img = mpimg.imread(image_path)

if class_id is not None:
    print("Direction detected :")
    if class_id == 3:
        classify = "Up"
        print("Up")
    elif class_id == 0:
        classify = "Down"
        print("Down")
    elif class_id == 2:
        classify = "Left"
        print("Left")
    else :
        classify = "Right"
        print("Right")
    
    print("Before showing image via Matplotlib")

    fig, ax = plt.subplots(figsize=(18,10))
    ax.imshow(img, cmap="gray")
    rect = patches.Rectangle((x1, y1), abs(x2-x1), abs(y2-y1), linewidth=1, edgecolor='k', facecolor='none')
    ax.add_patch(rect)

    fig.suptitle(classify, fontsize= 44)
    plt.show()
else:
    print("No Detections")
    fig, ax = plt.subplots()
    ax.imshow(img, cmap="gray")
    plt.show()

# file.close()