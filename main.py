import detection
from .hiding2 import HidingAlgorithm
# from rovio.rovio_video import IPCamera


def main():
    host = '192.168.43.18'
    hiding_algo = HidingAlgorithm(host)
    hiding_algo.loop()

def testvideo():
    import cv2
    model, classes = detection.init_model('./ssdweights/rovio_v2.h5')
    cap = cv2.VideoCapture('robotics2_obstacle.mp4')
    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            bboxes, disp_image = detection.predict(model, frame[:, :, ::-1], classes, 0.5)
            cv2.imshow('disp_image', disp_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

# import cv2
# camera = IPCamera('http://192.168.43.18/GetData.cgi?8935')
# while True:
#     frame = camera.get_frame()
#     cv2.imshow('test', frame)
#     k = cv2.waitKey(1) & 0xFF
#     if k == ord('q'):
#         break

if __name__ == '__main__':
    main()