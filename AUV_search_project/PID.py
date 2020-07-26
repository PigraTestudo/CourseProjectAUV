class PID:
    # инициализация
    def __init__(self, kp, ki, kd, maxI, maxU):
        self.Kp = kp
        self.Kd = kd
        self.Ki = ki
        self.lastErr = 0
        self.lastI = 0
        self.U = 0
        self.maxU = maxU
        self.maxI = maxI

    # основная функция
    def pidUpdate(self, Xd, X):
        err = Xd - X
        dt = 0.05
        P = self.Kp * err
        I = self.Ki * err * dt + self.lastI
        if I > self.maxI:
            I = self.maxI
        try:
            D = self.Kd / dt * (err - self.lastErr)
        except:
            D = 0
        self.lastErr = err
        self.lastI = I
        self.U = P + I + D
        if self.U > self.maxU:
            self.U = self.maxU
        if self.U < -self.maxU:
            self.U = -self.maxU
        return self.U
