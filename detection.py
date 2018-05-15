import os

import cv2
import numpy as np
from keras import backend as K
from keras.optimizers import Adam
from matplotlib import pyplot as plt

from keras_loss_function.keras_ssd_loss import SSDLoss
from models.keras_ssd300 import ssd_300


def non_max_suppression_fast(boxes, overlapThresh):
    if isinstance(boxes, list):
        boxes = np.array(boxes)
    # if there are no boxes, return an empty list
    if len(boxes) == 0:
        return []

    # if the bounding boxes integers, convert them to floats --
    # this is important since we'll be doing a bunch of divisions
    if boxes.dtype.kind == "i":
        boxes = boxes.astype("float")

    # initialize the list of picked indexes
    pick = []

    # grab the coordinates of the bounding boxes
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    # compute the area of the bounding boxes and sort the bounding
    # boxes by the bottom-right y-coordinate of the bounding box
    area = (x2 - x1 + 1) * (y2 - y1 + 1)
    idxs = np.argsort(y2)

    # keep looping while some indexes still remain in the indexes
    # list
    while len(idxs) > 0:
        # grab the last index in the indexes list and add the
        # index value to the list of picked indexes
        last = len(idxs) - 1
        i = idxs[last]
        pick.append(i)

        # find the largest (x, y) coordinates for the start of
        # the bounding box and the smallest (x, y) coordinates
        # for the end of the bounding box
        xx1 = np.maximum(x1[i], x1[idxs[:last]])
        yy1 = np.maximum(y1[i], y1[idxs[:last]])
        xx2 = np.minimum(x2[i], x2[idxs[:last]])
        yy2 = np.minimum(y2[i], y2[idxs[:last]])

        # compute the width and height of the bounding box
        w = np.maximum(0, xx2 - xx1 + 1)
        h = np.maximum(0, yy2 - yy1 + 1)

        # compute the ratio of overlap
        overlap = (w * h) / area[idxs[:last]]

        # delete all indexes from the index list that have
        idxs = np.delete(idxs, np.concatenate(([last],
                                               np.where(overlap > overlapThresh)[0])))

    # return only the bounding boxes that were picked using the
    # integer data type
    return boxes[pick].astype('int')


def init_model(weights_path='./ssdweights/rovio_v2.h5'):
    img_height = 300
    img_width = 300

    dirname = os.path.dirname(os.path.abspath(__file__))

    assert os.path.exists(weights_path), '%s not found...' % dirname

    K.clear_session()  # to clear all memory in the RAM

    model = ssd_300(image_size=(img_height, img_width, 3),
                    n_classes=2,
                    mode='inference_fast',
                    l2_regularization=0.0005,
                    scales=[0.1, 0.2, 0.37, 0.54, 0.71, 0.88, 1.05],
                    aspect_ratios_per_layer=[[1.0, 2.0, 0.5],
                                             [1.0, 2.0, 0.5, 3.0, 1.0 / 3.0],
                                             [1.0, 2.0, 0.5, 3.0, 1.0 / 3.0],
                                             [1.0, 2.0, 0.5, 3.0, 1.0 / 3.0],
                                             [1.0, 2.0, 0.5],
                                             [1.0, 2.0, 0.5]],
                    two_boxes_for_ar1=True,
                    steps=[8, 16, 32, 64, 100, 300],
                    offsets=[0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
                    limit_boxes=False,
                    variances=[0.1, 0.1, 0.2, 0.2],
                    coords='centroids',
                    normalize_coords=True,
                    subtract_mean=[123, 117, 104],
                    swap_channels=True,
                    confidence_thresh=0.5,
                    iou_threshold=0.45,
                    top_k=200,
                    nms_max_output_size=400)

    model.load_weights(weights_path, by_name=True)

    adam = Adam(lr=0.001, beta_1=0.9, beta_2=0.999, epsilon=1e-08, decay=5e-04)

    ssd_loss = SSDLoss(neg_pos_ratio=3, n_neg_min=0, alpha=1.0)

    model.compile(optimizer=adam, loss=ssd_loss.compute_loss)

    return model, ['background', 'rovio', 'rovio']


def predict(model, img, classes, confidence_threshold):
    img_height = 300
    img_width = 300
    orig_images = []  # Store the images here.
    input_images = []  # Store resized versions of the images here.

    img = cv2.imread(img)[:, :, ::-1] if isinstance(img, str) else img

    orig_images.append(img.copy())  # expect a cv2 rgb img
    if img.shape[:2] != (img_height, img_width):
        img = cv2.resize(img, (img_width, img_height))

    input_images.append(img)
    input_images = np.array(input_images)

    y_pred = model.predict(input_images)
    y_pred = [y_pred[k][y_pred[k, :, 1] > confidence_threshold] for k in range(y_pred.shape[0])]

    np.set_printoptions(precision=2, suppress=True, linewidth=90)
    # print("Predicted boxes:\n")
    # print('    class    conf  xmin    ymin    xmax    ymax')
    # print(y_pred[0])

    # Display the image and draw the predicted boxes onto it.

    # Set the colors for the bounding boxes
    colors = plt.cm.hsv(np.linspace(0, 1, 3)).tolist()
    colors = np.array(colors) * 255.0
    colors = colors[:, 1:].astype(np.uint8)
    # current_axis = plt.gca()
    boxes = []
    for box in y_pred[0]:
        # Transform the predicted bounding boxes for the 300x300 image to the original image dimensions.
        xmin = int(box[-4] * orig_images[0].shape[1] / img_width)
        ymin = int(box[-3] * orig_images[0].shape[0] / img_height)
        xmax = int(box[-2] * orig_images[0].shape[1] / img_width)
        ymax = int(box[-1] * orig_images[0].shape[0] / img_height)
        boxes.append((xmin, ymin, xmax, ymax, box[0]))

    boxes = non_max_suppression_fast(boxes, 0.3)

    ret_boxes = []
    for xmin, ymin, xmax, ymax, color in boxes:
        color = colors[int(color)]
        label = '{}: {:.2f}'.format(classes[int(box[0])], box[1])
        cv2.rectangle(orig_images[0], (xmin, ymin), (xmax, ymax), color.tolist())
        cv2.putText(orig_images[0], label, (xmin, ymin), cv2.FONT_HERSHEY_COMPLEX, 1, color.tolist())
        ret_boxes.append({'xmin': xmin, 'xmax': xmax, 'ymin': ymin, 'ymax': ymax, 'class_id': int(box[0])})

    return ret_boxes, orig_images[0][:, :, ::-1]


class RovioDetection(object):
    def __init__(self, screen_width=640, screen_height=480):
        self.weight_path = './ssdweights/rovio_v2.h5'
        self.model, self.classes = init_model(self.weight_path)
        self.boxes = []
        self.disp_img = None
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.cscreenx = self.screen_width / 2
        self.cscreeny = self.screen_height / 2

    def _get_rovio_bbox(self, frame):
        self.boxes, self.disp_img = predict(self.model, frame, self.classes, 0.7)
        return self.boxes

    def get_rovio_bbox(self, frame):
        boxes = self._get_rovio_bbox(frame)

        if len(boxes) == 0:
            return 'No Rovio Found'

        newboxes = []

        # preprocess
        for box in boxes:
            w = box['xmax'] - box['xmin']
            h = box['ymax'] - box['ymin']
            cx = box['xmin'] + w / 2
            cy = box['ymin'] + h / 2

            box['Area'] = w * h
            box['Refer_point'] = [cx, box['ymax']]
            box['Bottom_corner1'] = [box['xmin'], box['ymax']]
            box['Bottom_corner2'] = [box['xmax'], box['ymax']]
            box['center'] = [cx, cy]

            newboxes.append(box)

        # filter
        nearestbox = None

        mindistance = 0

        for box in boxes:
            cx, cy = box['center']
            # distance = math.sqrt((cx - self.cscreenx) ** 2 + (cy - self.cscreeny) ** 2)
            distance = abs(cx - self.cscreenx)
            box['distance_cscreenx'] = distance
            # no need compare for first box
            if nearestbox is None or mindistance > distance:
                nearestbox = box
                mindistance = distance

        return nearestbox

    def get_refer_point(self, frame):
        """

        if found:
            box = {
                'xmin': int,
                'xmax': int,
                'ymin': int,
                'ymax': int,
                'Area': int,
                'Refer_point': [x, y],
                'Bottom_corner1': [x, y],
                'Bottom_conrer2': [x, y],
                'center': [cx, cy],
                'distance_cscreenx': int,
                'direction': 1 or -1
            }
        else:
            'No Rovio Found'

        """
        frame = frame[:, :, ::-1]
        nearestbox = self.get_rovio_bbox(frame)

        if isinstance(nearestbox, str):
            return nearestbox

        cx, cy = nearestbox['center']

        if cx > self.cscreenx:
            nearestbox['direction'] = 1
        else:
            nearestbox['direction'] = -1

        return nearestbox


class ObstacleDetection(object):

    def __init__(self, bound=20):
        self.bound = 30

    def preprocess_image(self, frame):
        # 150,5,233
        pink = np.uint8([[[147, 20, 255]]])
        hsv_pink = cv2.cvtColor(pink, cv2.COLOR_BGR2HSV)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of pink color in HSV may try with +10 or -10
        lower_bound = np.array([hsv_pink[0][0][0] - self.bound, 147, 147])
        upper_bound = np.array([hsv_pink[0][0][0] + self.bound, 255, 255])

        # Threshold the HSV image to get only pink colors
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)

        # morphology
        kernel = np.ones((5, 5), np.uint8)
        opening = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        final_result = closing.copy()
        final_result = cv2.cvtColor(final_result, cv2.COLOR_RGB2GRAY)

        return final_result

    def get_refer_point(self, frame):
        final_result = self.preprocess_image(frame)
        img, cont, h = cv2.findContours(final_result.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        d = []
        rfx = 0
        rfy = 0
        btc1 = 0
        btc2 = 0
        if (len(cont) != 0):
            for i in range(len(cont)):
                area = cv2.contourArea(cont[i])
                if (area > 8000):  # why 8000? why if less than 8000?
                    cv2.drawContours(frame, cont[i], contourIdx=-1, color=(255, 0, 0), thickness=2, maxLevel=1)
                    x, y, w, h = cv2.boundingRect(cont[i])

                    f = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(f, "Obstacle", (x + 10, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
                    approx = cv2.approxPolyDP(cont[i], 0.05 * cv2.arcLength(cont[i], True), True)
                    ry_lst = []
                    bottom_corners = []
                    for a in range(len(approx)):
                        rx = approx[a][0][0]
                        ry = approx[a][0][1]
                        ry_lst.append(approx[a][0])

                        cv2.circle(f, (rx, ry), 5, 0, -1)
                        cv2.putText(f, ('(' + str(rx) + ',' + str(ry) + ')'), (rx - 40, ry + 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, True)

                    ry_lst = sorted(ry_lst, key=lambda k: k[1])
                    bottom_corners.append(ry_lst[-2:])
                    btc1 = [bottom_corners[0][0][0], bottom_corners[0][0][1]]
                    btc2 = [bottom_corners[0][1][0], bottom_corners[0][1][1]]
                    refer_point = [(bottom_corners[0][1][0] + bottom_corners[0][0][0]) / 2,
                                   (bottom_corners[0][1][1] + bottom_corners[0][0][1]) / 2]
                    rfx = int(refer_point[0])
                    rfy = int(refer_point[1])
                    cv2.circle(f, (rfx, rfy), 5, (0, 255, 255), -1)
                    cv2.putText(f, ('(' + str(rfx) + ',' + str(rfy) + ')'), (rfx - 40, rfy + 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, True)

                    d.append({'Area': area, 'Refer_point': [rfx, rfy], 'Bottom_corner1': btc1, 'Bottom_corner2': btc2})

                # d.append({'Area': area, 'Refer_point': [rfx, rfy], 'Bottom_corner1': btc1, 'Bottom_corner2': btc2})
                # return d

            if len(d) != 0:  # is it correct? if no area more than 8000, return No Obstacle Found
                return d

        return 'No Obstacle Found'
