from ultralytics import YOLO
import os
import csv

model_path = os.path.join('.', 'runs', 'detect', 'train', 'weights', 'last.pt')

model = YOLO(model_path) # load trained moel

test_path = "/home/csmith/Desktop/School/Winter 2024/Winter Project/ws/Winter-Project/yolo_model/dummy_arrow_tutorial/code - directions/data/images/test"
file = open("results.csv", 'w')

# writer = csv.DictWriter(file, fieldnames = ["num", "Class", "x1", "y1", "x2", "y2", "score"])
# writer.writeheader()

## get all files in the test files:
for image_file in os.listdir(test_path):
    image_path = os.path.join(test_path, image_file)
    # image_path = os.path.join(test_path, '1.jpg')
    # print(image_file)
    results = model.predict(image_path, conf=0.10)

    # print("########## My Print Statements #############")
    # print(type(results))  
    # print(results[0])
    # print("########## End My Print Statements #############")

    for result in results[0].boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = result
        image_num = int(image_file[:-4])

        # writer.writerow([str(image_num), str(class_id), str(x1), str(y1), str(x2), str(y2), str(score)])
        row = str(image_num)+', '+str(class_id)+', '+str(x1)+', '+str(y1)+', '+str(x2)+', '+str(y2)+', '+str(score)+'\n'
        file.write(row)

file.close()


