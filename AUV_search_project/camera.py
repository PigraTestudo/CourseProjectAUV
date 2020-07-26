# ========== библиотеки ================================================================================================
import vrep
import vrepConst
import numpy as np
import cv2


class Camera:

    # инициализация
    def __init__(self, clientid, handle):
        self.clientID = clientid
        self.img = None
        self.thresh = None
        self.auv = None

        self.obj_coords = [0.0, 0.0, 0.0]
        self.obj_coords_l = [0.0, 0.0, 0.0]
        self.obj_list = []
        self.obj_count = 0

        res, self.cameraHandle = vrep.simxGetObjectHandle(self.clientID, handle, vrep.simx_opmode_blocking)
        res, self.resolution, image = vrep.simxGetVisionSensorImage(self.clientID, self.cameraHandle, 0,
                                                                    vrep.simx_opmode_streaming)

    # получение данных с камеры
    def getCameraImage(self):
        res, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, self.cameraHandle, 0,
                                                               vrep.simx_opmode_buffer)
        image = np.array(image, dtype=np.uint8)
        image.resize([resolution[1], resolution[0], 3])
        center = (resolution[0] / 2, resolution[1] / 2)
        M = cv2.getRotationMatrix2D(center, -90, 1.0)
        rotImg = cv2.warpAffine(image, M, (resolution[0], resolution[1]))
        normImg = cv2.flip(rotImg, 1)
        self.img = np.flip(normImg, axis=2)

        # цветофильтры
        # img_rgb = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        # hsv_min = np.array((50, 0, 0), np.uint8)          # 50 0 0 | 255 255 255 green
        # hsv_max = np.array((255, 255, 255), np.uint8)
        # hsv_min = np.array((30, 0, 0), np.uint8)          # 30 0 0 | 40  255 255 yellow (inverse)
        # hsv_max = np.array((40, 255, 255), np.uint8)
        hsv_min = np.array((5, 0, 0), np.uint8)             # 5  0 0 | 255 255 255 red
        hsv_max = np.array((255, 255, 255), np.uint8)

        self.img2 = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
        self.hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        self.thresh = cv2.inRange(self.hsv, hsv_min, hsv_max)

    # распознавание объектов
    def objectsRecon(self, XYZ, ABG):
        self.getCameraImage()
        circles = cv2.HoughCircles(self.thresh, cv2.HOUGH_GRADIENT, 1, 15, param1=30, param2=15, minRadius=5, maxRadius=50)
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.rectangle(self.img2, (x - 2, y - 2), (x + 2, y + 2), (0, 0, 0), -1)
        contours, hierarchy = cv2.findContours(self.thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        x_obj = 0
        y_obj = 0
        if len(contours) > 0:
            cv2.drawContours(self.img2, contours, -1, (255, 255, 255), 1)
            if len(contours[0]) > 0:
                M = cv2.moments(contours[0])
                if 30000 > M["m00"] > 1000:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(self.img2, (cX, cY), 3, (255, 0, 255), -1)

                    # == здесь будут координаты объекта
                    x_obj = cX * np.cos(ABG[2])/256  - cX * np.sin(ABG[2])/256
                    y_obj = cY * np.sin(ABG[2])/256 + cY * np.cos(ABG[2])/256
        if x_obj != 0:
            if y_obj != 0:
                x_obj = x_obj + XYZ[0]
                y_obj = y_obj + XYZ[0]
                self.obj_coords = [x_obj, y_obj, XYZ[2]]
                if (abs(self.obj_coords_l[0] - x_obj) > 1) or (abs(self.obj_coords_l[1] - y_obj) > 1):
                    self.obj_list.append(self.obj_coords)
                    self.obj_coords_l = self.obj_coords
                    self.obj_count += 1

        print("|    there are ", self.obj_count, "objects")
        print("|    their coordinates are: ", self.obj_list)
        print("|----------------------------------|")

    # открытие стрима
    def showStream(self):
        cv2.imshow('image', self.img2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

    # закрытие стрима
    def closeAllStreams(self):
        cv2.destroyAllWindows()

    # основная функция
    def stateOfSearch(self, XYZ, auv):
        self.auv = auv
        res, ABG = vrep.simxGetObjectOrientation(self.clientID, self.auv, -1, vrep.simx_opmode_buffer)
        self.objectsRecon(XYZ, ABG)
        self.showStream()
