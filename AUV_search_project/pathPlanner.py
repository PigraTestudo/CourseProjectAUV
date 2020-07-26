# ========== библиотеки ================================================================================================
import vrep
import vrepConst
import time
import numpy as np
import sys

# ========== модули ====================================================================================================
from PID import PID as PIDc
from proximitySensor import ProxSensor as PSens


class PathPlanner:
    # инициализация
    def __init__(self, clientID, auv, vel):
        self.clientID = clientID
        self.auv = auv
        self.Plan = []
        self.stage = 0
        self.path_stages = 0
        self.Velocity = vel * 2500
        self.XYZ = [0, 0, 2]
        self.XYZ_d = [0, 0, 0]
        self.ABG = vrep.simxGetObjectOrientation(clientID, auv, -1, vrep.simx_opmode_streaming)
        self.ABG_d = [0, 0, 0]
        self.timeStart = 0
        self.timeEnd = time.time()
        self.sensors = PSens(self.clientID)
        self.heightSensor = self.sensors.heightSensor()
        self.setPIDc()

    def setPIDc(self):
        # параметры ПИДов
        self.fwd_ctrl        = PIDc(777, 0, 1, 1000, self.Velocity)  # контроль движения вперед
        self.rotation_ctrl   = PIDc(50, 0, 5, 1000, 1000)   # контроль рыскания

        self.stab_ctrl       = PIDc(1000, 0, 0, 2500, 5000)   # контроль высоты
        self.pitchroll_ctrl  = PIDc(25, 0, 0, 100, 100)   # контроль крена и тангажа

    # нахождение своих координат
    def coordinates(self):

        res, self.ABG = vrep.simxGetObjectOrientation(self.clientID, self.auv, -1, vrep.simx_opmode_buffer)
        Speed = vrep.simxGetObjectVelocity(self.clientID, self.auv, vrep.simx_opmode_streaming)
        speedLinear = Speed[1]
        dt = self.timeEnd - self.timeStart
        if dt < 1.0:
            self.XYZ[0] += speedLinear[0] * dt
            self.XYZ[1] += speedLinear[1] * dt
            self.XYZ[2] += speedLinear[2] * dt
        return self.XYZ

    # рассчет сил
    def calcTheForce(self):
        # расчет сил
        dx = self.XYZ_d[0] - self.XYZ[0]
        dy = self.XYZ_d[1] - self.XYZ[1]

        self.ABG_d[2] = np.arctan2(dy, dx)

        Fx_d = 0
        if -0.05 <= self.ABG_d[2] - self.ABG[2] <= 0.05:
            if dx < 0 and dy < 0:
                if dx > dy:
                    Fx_d = self.fwd_ctrl.pidUpdate(self.XYZ_d[0], self.XYZ[0]) * -1
                else:
                    Fx_d = self.fwd_ctrl.pidUpdate(self.XYZ_d[1], self.XYZ[1]) * -1
            if dx > 0 and dy > 0:
                if dx > dy:
                    Fx_d = self.fwd_ctrl.pidUpdate(self.XYZ_d[0], self.XYZ[0])
                else:
                    Fx_d = self.fwd_ctrl.pidUpdate(self.XYZ_d[1], self.XYZ[1])
            if dx < 0 and dy > 0:
                if dx > dy:
                    Fx_d = self.fwd_ctrl.pidUpdate(self.XYZ_d[0], self.XYZ[0]) * -1
                else:
                    Fx_d = self.fwd_ctrl.pidUpdate(self.XYZ_d[1], self.XYZ[1])
            if dx > 0 and dy < 0:
                if dx > dy:
                    Fx_d = self.fwd_ctrl.pidUpdate(self.XYZ_d[0], self.XYZ[0])
                else:
                    Fx_d = self.fwd_ctrl.pidUpdate(self.XYZ_d[1], self.XYZ[1]) * -1
            # перемещения вдоль игрек
            if -0.5 <= dx <= 0.5 and (self.XYZ[1] <= self.XYZ_d[1]):
                Fx_d = self.fwd_ctrl.pidUpdate(self.XYZ_d[1], self.XYZ[1])
            if (-0.5 <= dx <= 0.5) and (self.XYZ[1] > self.XYZ_d[1]):
                Fx_d = self.fwd_ctrl.pidUpdate(self.XYZ_d[1], self.XYZ[1]) * -1
            # перемещения вдоль икс
            if -0.5 <= dy <= 0.5 and (self.XYZ[0] <= self.XYZ_d[0]):
                Fx_d = self.fwd_ctrl.pidUpdate(self.XYZ_d[0], self.XYZ[0])
            if (-0.5 <= dy <= 0.5) and (self.XYZ[0] > self.XYZ_d[0]):
                Fx_d = self.fwd_ctrl.pidUpdate(self.XYZ_d[0], self.XYZ[0]) * -1
        else:
            Fx_d = 0
        Fy_d = 0
        Fz_d = PIDc.pidUpdate(self.stab_ctrl,      self.XYZ_d[2], self.XYZ[2])

        # повороты: рыскание, тангаж и крен
        Mx_d = PIDc.pidUpdate(self.rotation_ctrl,  self.ABG_d[2], self.ABG[2])  # вокруг Z, рыскание
        My_d = PIDc.pidUpdate(self.pitchroll_ctrl, self.ABG_d[1], self.ABG[1])
        Mz_d = PIDc.pidUpdate(self.pitchroll_ctrl, self.ABG_d[0], self.ABG[0])

        forceD = [Fx_d, Fy_d, Fz_d, Mx_d, My_d, Mz_d]
        s_force = vrep.simxPackFloats(forceD)
        vrep.simxSetStringSignal(self.clientID, "ForceD", s_force, vrep.simx_opmode_oneshot)

        # print("Force XYZ = ", Fx, Fy, Fz)
        # print("Force ABG = ", Rx, Ry, Rz)

    # передача сил движителям
    def useTheForce(self, Fx, Fy, Fz, Rx, Ry, Rz):
        forceD = [Fx, Fy, Fz, Rx, Ry, Rz]
        s_force = vrep.simxPackFloats(forceD)
        res = vrep.simxSetStringSignal(self.clientID, "ForceD", s_force, vrep.simx_opmode_oneshot)

    # установка параметров полигона
    def setPolygon(self, height, Xside, Yside, vel):
        self.height = height
        self.Xside = Xside
        self.Yside = Yside
        self.Velocity = vel * 2500

    # планировщик маршрута
    def makePath(self):
        sx = self.Xside / 2
        ex =-self.Xside / 2
        sy = self.Yside / 2
        state = 1
        self.XYZ_d = (0, sy, self.height)
        self.Plan.append(self.XYZ_d)
        for iter in np.linspace(sx, ex, self.Xside + 1):
            self.XYZ_d = (iter, sy * state, self.height)
            self.Plan.append(self.XYZ_d)
            state *= -1
            self.XYZ_d = (iter, sy * state, self.height)
            self.Plan.append(self.XYZ_d)
        self.path_stages = 2 * self.Xside + 4
        self.XYZ_d = (0.0, 0.0, 1.0)
        self.Plan.append(self.XYZ_d)
        self.Plan.append(self.XYZ_d)

        print("|----------------------------------|")
        print("|    Number of stages is [", self.path_stages, "]")
        for i in range(0, self.path_stages, 1):
            print("|    ", i + 1, ":", self.Plan[i])
        print("|----------------------------------|")

    # следование по маршруту
    def followPath(self):
        self.XYZ_d = list(self.Plan[self.stage])
        X = self.XYZ[0] - self.XYZ_d[0]
        Y = self.XYZ[1] - self.XYZ_d[1]
        Z = self.XYZ[2] - self.XYZ_d[2]

        if (-0.1 < X < 0.1) and (-0.1 < Y < 0.1):
            self.stage += 1
            self.XYZ_d = list(self.Plan[self.stage])

        print("|    stage  [", self.stage, "]")
        print("|    goal   [", '{:04.2f}'.format(self.XYZ_d[0]), '{:04.2f}'.format(self.XYZ_d[1]), '{:04.2f}'.format(self.XYZ_d[2]), "]")
        print("|    now at [", '{:04.2f}'.format(self.XYZ[0]), '{:04.2f}'.format(self.XYZ[1]), '{:04.2f}'.format(self.XYZ[2]), "]")
        print("|----------------------------------|")

        if self.stage == self.path_stages:
            print("|        MISSION  COMPLETED        |")
            print("|----------------------------------|")
            sys.exit()

    # выдача собственных координат
    def returnCoord(self):
        return self.XYZ

    # основная функция
    def stateOfMove(self):
        self.useTheForce(0, 0, 0, 0, 0, 0)
        self.timeStart = self.timeEnd
        self.timeEnd = time.time()
        self.XYZ = self.coordinates()
        self.heightSensor = self.sensors.heightSensor()
        self.XYZ[2] = self.heightSensor
        self.XYZ = list(self.XYZ)
        self.calcTheForce()
        self.followPath()
