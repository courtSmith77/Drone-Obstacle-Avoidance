import imutils
import cv2
import numpy as np

def order_corner_points(corners):
    # Separate corners into individual points
    # Index 0 - top-right
    #       1 - top-left
    #       2 - bottom-left
    #       3 - bottom-right
    # corners = [(corner[0][0], corner[0][1]) for corner in corners]
    top_r, top_l, bottom_l, bottom_r = corners[0], corners[1], corners[2], corners[3]
    return (top_l, top_r, bottom_r, bottom_l)

def perspective_transform(image, corners):

    # Order points in clockwise order
    ordered_corners = order_corner_points(corners)
    top_l, top_r, bottom_r, bottom_l = ordered_corners

    # Determine width of new image which is the max distance between 
    # (bottom right and bottom left) or (top right and top left) x-coordinates
    width_A = np.sqrt(((bottom_r[0] - bottom_l[0]) ** 2) + ((bottom_r[1] - bottom_l[1]) ** 2))
    width_B = np.sqrt(((top_r[0] - top_l[0]) ** 2) + ((top_r[1] - top_l[1]) ** 2))
    width = max(int(width_A), int(width_B))

    # Determine height of new image which is the max distance between 
    # (top right and bottom right) or (top left and bottom left) y-coordinates
    height_A = np.sqrt(((top_r[0] - bottom_r[0]) ** 2) + ((top_r[1] - bottom_r[1]) ** 2))
    height_B = np.sqrt(((top_l[0] - bottom_l[0]) ** 2) + ((top_l[1] - bottom_l[1]) ** 2))
    height = max(int(height_A), int(height_B))

    # Construct new points to obtain top-down view of image in 
    # top_r, top_l, bottom_l, bottom_r order
    dimensions = np.array([[0, 0], [width - 1, 0], [width - 1, height - 1], 
                    [0, height - 1]], dtype = "float32")

    # Convert to Numpy format
    ordered_corners = np.array(ordered_corners, dtype="float32")

    # Find perspective transform matrix
    matrix = cv2.getPerspectiveTransform(ordered_corners, dimensions)

    # Return the transformed image
    transformed_image = cv2.warpPerspective(image, matrix, (width, height))

    return transformed_image

def detect_arrow(model, img):

    results = model.predict(img)

    # detected = None
    # for result in results[0].boxes.data.tolist():
    #     x1, y1, x2, y2, score, detected = result

    biggest_class = None
    area = 0
    boxes = []
    classes = []
    for result in results[0].boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = result
        box_area = abs((x2-x1)*(y2-y1))
        # boxes.append([x1,y1,x2,y2])
        # classes.append(class_id)
        if box_area > area:
            area = box_area
            biggest = [x1,y1,x2,y2]
            biggest_class = class_id
    

    if biggest_class is not None:
        return biggest
    else :
        return None
    
def classify_direction(model, img, box):

    yolo_corners = [[box[0],box[3]],[box[2],box[3]],[box[2],box[1]],[box[0],box[1]]] # top_l, top_r, bot_r, bot_l

    transformed = perspective_transform(img, yolo_corners)
    flipped = cv2.flip(transformed, -1)
    img_depth = cv2.convertScaleAbs(flipped)

    pred = model.predict(img_depth)

    class_id = None
    if pred is not None:
        class_id = pred[0].probs.top1

    return class_id

def important_edge(classify, box):

        # 0,1,2,3 (down,left,right,up)
        if classify == 0:
            return box[3]
        elif classify == 1:
            return box[0]
        elif classify == 2:
            return box[2]
        elif classify == 3:
            return box[3]

def control_cmds(area, edge, classify):

    if area > 45000:
        fly_forward = 0
    else:
        fly_forward = 30
    
    if area < 20000:
        fly_dist = 0
    else :        
        if classify == 0:
            dist_from_center = 360 - edge
            print(f"Distance from Center = {dist_from_center}")
            if dist_from_center < 200:
                fly_dist = 25
            else:
                fly_dist = 0

        elif classify == 1:
            dist_from_center = 640 - edge
            print(f"Distance from Center = {dist_from_center}")
            if dist_from_center < 200:
                fly_dist = 25
            else:
                fly_dist = 0

        elif classify == 2:
            dist_from_center = 640 - edge
            print(f"Distance from Center = {dist_from_center}")
            if dist_from_center > -200:
                fly_dist = 30
            else:
                fly_dist = 0

        elif classify == 3:
            dist_from_center = 360 - edge
            print(f"Distance from Center = {dist_from_center}")
            if dist_from_center > -100:
                fly_dist = 25
            else:
                fly_dist = 0
    
    return fly_forward, fly_dist