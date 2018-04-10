import cv2
import requests
import numpy as np
import urllib


class IPCamera(object):
    def __init__(self, url):
        self.url = url
        self._frame_iter = self._get_frame_iterator()

    def _get_frame_iterator(self):
        stream = urllib.urlopen(self.url)
        bytes = ''
        while True:
            bytes += stream.read(1024)
            a = bytes.find('\xff\xd8')
            b = bytes.find('\xff\xd9')
            if a != -1 and b != -1:
                jpg = bytes[a:b + 2]
                bytes = bytes[b + 2:]
                frame = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), 1)
                yield frame

    def get_frame(self):
        ret, frame = True, self._frame_iter.next()
        return ret, frame
