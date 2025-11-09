import numpy as np

class UWBKalmanFilter:
    """
    簡化版 3D 卡爾曼濾波器
    狀態: [x, y, z, vx, vy, vz]
    觀測: [x, y, z]
    """

    def __init__(self, dt=0.1, process_var=1e-2, measurement_var=5e-1):
        self.dt = dt

        # 狀態向量: [x, y, z, vx, vy, vz]
        self.x = np.zeros((6, 1))

        # 狀態轉移矩陣 F
        self.F = np.eye(6)
        for i in range(3):
            self.F[i, i + 3] = dt

        # 觀測矩陣 H
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.H[2, 2] = 1

        # 狀態協方差矩陣 P
        self.P = np.eye(6) * 500.0

        # 過程噪聲 Q、量測噪聲 R
        self.Q = np.eye(6) * process_var
        self.R = np.eye(3) * measurement_var

    # ------------------------------------------------------------
    def predict(self):
        """預測下一時刻狀態"""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:3].flatten()

    # ------------------------------------------------------------
    def update(self, z_measured):
        """更新觀測"""
        z = np.array(z_measured).reshape(3, 1)
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P
        return self.x[:3].flatten()

    # ------------------------------------------------------------
    def filter(self, z_measured):
        """預測 + 更新"""
        self.predict()
        return self.update(z_measured)
