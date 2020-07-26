# ========== библиотеки ================================================================================================
import vrep
import vrepConst
import time

# ========== модули ====================================================================================================
from pathPlanner    import PathPlanner      as PP
from camera         import Camera           as cam
from userInterface  import UserInterface as cUI

cui = cUI()  # интерфейс создается сразу

# ========== инициализация =============================================================================================

h, Xside, Yside, vel = cui.setStuff()
vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 15000, 5)
if clientID != -1:
    print('|    Connected to AUV              |')
res, auv = vrep.simxGetObjectHandle(clientID, "Sphere", vrep.simx_opmode_oneshot_wait)

# создание объектов классов
path    = PP(clientID, auv, vel)
camera  = cam(clientID, "Vision_sensor")

# отправка нулевых сил
forceD = [0, 0, 0, 0]
s_force = vrep.simxPackFloats(forceD)
vrep.simxSetStringSignal(clientID, "ForceD", s_force, vrep.simx_opmode_oneshot)

# планировщик маршрута
path.setPolygon(h, Xside, Yside, vel)
path.makePath()

# ========== начало работы =============================================================================================
time.sleep(1)
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
print("|              start               |")
print("|                                  |")

while True:
    path.stateOfMove()
    camera.stateOfSearch(path.returnCoord(), auv)
    time.sleep(0.05)
