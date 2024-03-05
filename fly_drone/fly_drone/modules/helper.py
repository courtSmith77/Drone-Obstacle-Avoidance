import numpy as np
import cv2

def order_corner_points(corners):
    """Separates corners into individual points.
    
    Inputs:
        corners - the detection box
    Returns:
        ordered_corners - puts the corners in teh desired order
    """
    top_r, top_l, bottom_l, bottom_r = corners[0], corners[1], corners[2], corners[3]
    return (top_l, top_r, bottom_r, bottom_l)

def perspective_transform(image, corners):
    """Crops the image to the given corner locations.
    
    Inputs:
        image - the live feed image
        corners - the detection box
    Returns:
        transformed_image - the cropped image
    """
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
    """Detects arrow within the image.
    
    Inputs:
        model - the yolo detection model
        img - the live feed image
    Returns:
        biggest - the coordinates of the detection box of the bottom left (x1,y1)
                and top right (x2,y2) or None if nothing detected
    """
    results = model.predict(img)

    # detected = None
    # for result in results[0].boxes.data.tolist():
    #     x1, y1, x2, y2, score, detected = result

    biggest_class = None
    area = 0
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
    """Classifies detected arrow.
    
    Inputs:
        model - the yolo arrow classification model
        img - the live feed image
        box - the detection box
    Returns:
        class_id - the class id number 0,1,2,3 corresponding to direction down, left, right, up respectively
    """
    yolo_corners = [[box[0],box[3]],[box[2],box[3]],[box[2],box[1]],[box[0],box[1]]] # top_l, top_r, bot_r, bot_l

    # crop the image to just what is within the detection box
    transformed = perspective_transform(img, yolo_corners)
    flipped = cv2.flip(transformed, -1)
    img_depth = cv2.convertScaleAbs(flipped)

    pred = model.predict(img_depth)

    class_id = None
    if pred is not None:
        class_id = pred[0].probs.top1

    return class_id

def important_edge(classify, box):
    """Defines the important edge for each arrow direction.
    
    Inputs:
        classify - arrow class id
        box - the detection box
    Returns:
        edge - the important edge location
    """
    # 0,1,2,3 (down,left,right,up)
    if classify == 0:
        return box[3]
    elif classify == 1:
        return box[0]
    elif classify == 2:
        return box[2]
    elif classify == 3:
        return box[3]

def symbol_center(box):
    """Calculates the center of the symbol.
    
    Inputs:
        box - the detection box
    Returns:
        xc - the x center of the symbol
        yc - the y center of the symbol
    """
    xc = box[0] + 0.5*(box[2] - box[0])
    yc = box[1] + 0.5*(box[3] - box[1])

    return xc, yc

def control_cmds(self, area, edge, x_c, y_c, classify):
    """Calculates the control commands to move the drone.
    
    Inputs:
        self - node instance to log statements
        area - the area of the detection box
        edge - the important edge based on class id
        x_c - x center of the symbol 
        y_c - y center of the symbol
        classify - the class id of the symbol
    Returns:
        fly_forward - the flight distance forward
        fly_dist - the flight distance in the direction of the symbol
        ortho_dist - the flight distance orthogonal to the symbol direction
    """
    if area > 45000:
        fly_forward = 0
    else:
        fly_forward = 30
    
    if area < 15000:
        fly_dist = 0
        ortho_dist = 0
    else :        
        if classify == 0:
            dist_from_center = 360 - edge
            self.get_logger().info(f"Distance from Center = {dist_from_center}")
            fly_dist = 20

            # determine orthogonal movement
            if abs(x_c - 640) > 200 :
                ortho_dist = int(-20*(x_c - 640)/abs(x_c - 640))
            else :
                ortho_dist = 0

        elif classify == 1:
            dist_from_center = 640 - edge
            self.get_logger().info(f"Distance from Center = {dist_from_center}")
            fly_dist = 20

            # determine orthogonal movement
            if abs(y_c - 360) > 100 :
                ortho_dist = int(-20*(y_c - 360)/abs(y_c - 360))
            else :
                ortho_dist = 0

        elif classify == 2:
            dist_from_center = 640 - edge
            self.get_logger().info(f"Distance from Center = {dist_from_center}")
            fly_dist = 20
            
            # determine orthogonal movement
            if abs(y_c - 360) > 100 :
                ortho_dist = int(-20*(y_c - 360)/abs(y_c - 360))
            else :
                ortho_dist = 0

        elif classify == 3:
            dist_from_center = 360 - edge
            self.get_logger().info(f"Distance from Center = {dist_from_center}")
            fly_dist = 20

            # determine orthogonal movement
            if abs(x_c - 640) > 200 :
                ortho_dist = int(-20*(x_c - 640)/abs(x_c - 640))
            else :
                ortho_dist = 0
    
    return fly_forward, fly_dist, ortho_dist

def special_cmds(self, area, x_c, y_c):
    """Calculates the control commands to move the drone.
    
    Inputs:
        self - node instance to log statements
        area - the area of the detection box
        x_c - x center of the symbol 
        y_c - y center of the symbol
    Returns:
        forward_dist - the flight distance forward
        x_dist - the flight distance in the horizontal direction
        y_dist - the flight distance in the vertical direction
    """
    if (area > 100000) or (area < 50000):
        forward_dist = 30
    else:
        forward_dist = 0
    
    if (area < 15000) or (area > 50000):
        x_dist = 0
        y_dist = 0
    else: # center on the robot       
        self.get_logger().info(f"x-dist from center = {x_c-360}")
        self.get_logger().info(f"y-dist from center = {y_c-360}")

        if abs(y_c - 360) > 100 :
            y_dist = int(-30*(y_c - 360)/abs(y_c - 360))
        else :
            y_dist = 0

        if abs(x_c - 640) > 400 :
            x_dist = int(-30*(x_c - 640)/abs(x_c - 640))
        else :
            x_dist = 0
    
    return forward_dist, x_dist, y_dist

