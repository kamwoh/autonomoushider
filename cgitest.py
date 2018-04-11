import cv2

from .rovio.rovio_video import IPCamera

camera = IPCamera('http://24.103.196.243:80/cgi-bin/viewer/video.jpg?r=1523382467', debug=False)

while True:
    frame = None
    while frame is None:
        ret, frame = camera.get_frame()

    cv2.imshow('test', frame)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break