import vrep


class ProxSensor:
    # инициализация
    def __init__(self, clientID):
        self.clientID = clientID
        self.F_fwd = 1
        self.F_right = 1
        self.F_left = 1
        self.dtctd_height = 1

    # обновление данных сенсоров
    def updateSensor(self):

        self.F_fwd      = self.genericSensor("proxens_fwd")
        self.F_right    = self.genericSensor("proxens_right")
        self.F_left     = self.genericSensor("proxens_left")

        return self.F_fwd, self.F_right, self.F_left

    # сенсор расстояния от дна
    def heightSensor(self):
        # ====== сенсор высоты =========================================================================================
        res, proxensor = vrep.simxGetObjectHandle(self.clientID, "proxens_height", vrep.simx_opmode_blocking)
        res = vrep.simxReadProximitySensor(self.clientID, proxensor, vrep.simx_opmode_oneshot)
        if res[1] is True:
            self.detectedCoord = res[2]
            self.dtctd_height = self.detectedCoord[2]
        return self.dtctd_height

    # общая функция для обновления
    def genericSensor(self, name):
        res, proxensor = vrep.simxGetObjectHandle(self.clientID, name, vrep.simx_opmode_blocking)
        res = vrep.simxReadProximitySensor(self.clientID, proxensor, vrep.simx_opmode_oneshot)
        if res[1] is True:
            force = -2
        else:
            force = 1
        return force
