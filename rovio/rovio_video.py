import cv2
import numpy as np
import requests

class IPCamera(object):
    def __init__(self, url):
        self.url = url

    def get_frame(self):
        response = requests.get(self.url)
        print response
        img_array = np.asarray(bytearray(response.content), dtype=np.uint8)
        frame = cv2.imdecode(img_array, 1)
        return frame