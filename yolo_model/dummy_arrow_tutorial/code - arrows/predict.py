from ultralytics import YOLO
import os
import csv

model_path = os.path.join('.', 'runs', 'detect', 'train', 'weights', 'last.pt')

model = YOLO(model_path) # load trained moel

test_path = "/home/csmith/Desktop/School/Winter 2024/Winter Project/ws/Winter-Project/yolo_model/dummy_arrow_tutorial/code/data/images/test"
file = open("results.csv", 'w')

writer = csv.DictWriter(file, fieldnames = ["Class", "x1", "y1", "x2", "y2", "score"])
writer.writeheader()

## get all files in the test files:
# for image_file in os.listdir(test_path):
# image_path = os.path.join(test_path, image_file)
image_path = os.path.join(test_path, '2.jpg')
# print(image_file)
results = model(image_path)

print("########## My Print Statements #############")
print(type(results))
print(results[0])
print("########## End My Print Statements #############")

    # for result in results.boxes.data.tolist():
    #     x1, y1, x2, y2, score, class_id = result
    #     image_num = int(image_file[:-4])

    #     writer.writerow([image_num, class_id, x1, y1, x2, y2, score])


