class UserInterface:
    # инициализация
    def __init__(self):
        self.h = 0
        self.Xside = 0
        self.Yside = 0
        self.vel = 0

    # ввод расстояния от дна
    def setHeight(self):
        while self.h == 0:
            print("|    set the distance from bottom  |")
            print("|    -- note: 2 m is recommended   |")
            self.res = float(input("|    h = "))
            if 0 < self.res < 4:
                self.h = self.res
                print("|    got it!                       |")
                print("|----------------------------------|")
            else:
                print("|    wrong input                   |")

    # ввод длины
    def setPolygonX(self):
        while self.Xside == 0:
            self.res = 0
            print("|    set the distance X side       |")
            print("|    -- note: integer only         |")
            self.res = int(input("|    X side = "))
            if 0 < self.res < 300:
                self.Xside = self.res
                print("|    got it!                       |")
                print("|----------------------------------|")
            else:
                print("|    wrong input                   |")

    # ввод ширины
    def setPolygonY(self):
        while self.Yside == 0:
            self.res = 0
            print("|    set the distance Y side       |")
            print("|    -- note: integer only         |")
            self.res = int(input("|    Y side = "))
            if 0 < self.res < 300:
                self.Yside = self.res
                print("|    got it!                       |")
                print("|----------------------------------|")
            else:
                print("|    wrong input                   |")

    def setVelocity(self):
        while self.vel == 0:
            self.res = 0
            print("|    set the velocity              |")
            print("|    -- note: 1-2 m/s is okay      |")
            self.res = int(input("|    Velocity = "))
            if 0 < self.res < 5:
                self.vel = self.res
                print("|    got it!                       |")
                print("|----------------------------------|")
            else:
                print("|    wrong input                   |")
    #
    def setStuff(self):
        print("|----------------------------------|")
        print("|     welcome to the AUV ICS's     |\n"
              "|      minimalistic interface      |\n")
        self.setHeight()
        self.setPolygonX()
        self.setPolygonY()
        self.setVelocity()
        print("|          polygon is set!         |")
        input("|  press ENTER to make it happen!  |\n")

        return self.h, self.Xside, self.Yside, self.vel
