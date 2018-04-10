import cv2

from rovio.rovio_video import IPCamera

camera = IPCamera('http://192.168.43.18/GetData.cgi?8935')

while True:
    frame = None
    while frame is None:
        ret, frame = camera.get_frame()

    cv2.imshow('test', frame)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break
