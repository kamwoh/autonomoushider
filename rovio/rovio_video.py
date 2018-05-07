import requests
import threading

import cv2
import numpy as np


class IPCamera(object):
    def __init__(self, url, debug=True):
        self.url = url
        self.current_frame = None
        self.debug = debug
        self.thread = threading.Thread(target=self._get_frame)
        self.thread.setDaemon(True)
        self.thread.start()

    def _get_frame(self):
        r = requests.get(self.url, stream=True)
        _iter = r.iter_content(chunk_size=1024)
        _bytes = bytes()
        while True:
            try:
                chunk = next(_iter)
                _bytes += chunk
                a = _bytes.find(b'\xff\xd8')
                b = _bytes.find(b'\xff\xd9')
                if a != -1 and b != -1:
                    jpg = _bytes[a:b + 2]
                    _bytes = _bytes[b + 2:]
                    i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    self.current_frame = cv2.resize(i, (640, 480))
                    if self.debug:
                        i = cv2.resize(i, (480,360))
                        cv2.imshow('debug', i)
                        cv2.moveWindow('debug', 50, 100)
                        cv2.waitKey(1)
            except Exception as e:
                print ('gg error: ' + str(e))

    def get_frame(self):
        ret = True
        frame = self.current_frame.copy() if self.current_frame is not None else None
        return ret, frame
