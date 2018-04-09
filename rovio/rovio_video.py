import cv2


class IPCamera(object):
    def __init__(self, url):
        self.url = url

    def _get_frame_iterator(self):
        self.cap = cv2.VideoCapture(self.url)
        while True:
            yield self.cap.read()

    def get_frame(self):
        ret, frame = self._get_frame_iterator().next()
        return ret, frame
