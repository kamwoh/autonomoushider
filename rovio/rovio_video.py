import threading

import cv2
import numpy as np
import requests


class IPCamera(object):
    def __init__(self, url, debug=True):
        self.url = url
        self.current_frame = None
        self.debug = debug
        self.thread = threading.Thread(target=self._get_frame, verbose=1)
        self.thread.setDaemon(True)
        self.thread.start()

    def _get_frame(self):
        r = requests.get(self.url, stream=True)
        _iter = r.iter_content(chunk_size=1024)
        _bytes = bytes()
        while True:
            try:
                chunk = _iter.next()
                _bytes += chunk
                a = _bytes.find(b'\xff\xd8')
                b = _bytes.find(b'\xff\xd9')
                if a != -1 and b != -1:
                    jpg = _bytes[a:b + 2]
                    _bytes = _bytes[b + 2:]
                    i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    self.current_frame = i
                    if self.debug:
                        cv2.imshow('debug', self.current_frame)
                        cv2.waitKey(1)
            except Exception as e:
                print ('gg error: ' + str(e.message))

    def get_frame(self):
        ret = True
        frame = self.current_frame.copy() if self.current_frame is not None else None
        return ret, frame
