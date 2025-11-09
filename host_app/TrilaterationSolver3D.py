import numpy as np

class TrilaterationSolver3D:
    """
    通用三維 / 已知 Z 的 UWB 定位求解器

    支援：
      - 3D Trilateration（未知 z，需要 ≥4 anchors）
      - 已知 Z 化簡求解（需要 ≥3 anchors）
    """

    def __init__(self, anchors):
        """
        anchors: list[tuple(float, float, float)]
            anchor 座標，例如 [(x1, y1, z1), (x2, y2, z2), ...]
        """
        self.anchors = np.array(anchors, dtype=float)
        if self.anchors.shape[1] != 3:
            raise ValueError("Anchors 必須為 3 維座標 (x, y, z)。")

    # ------------------------------------------------------------
    def solve(self, distances, known_z=None):
        """
        計算 Tag 的座標。

        參數:
            distances : list[float]
                量測距離 (m)
            known_z : float | None
                若指定已知 z，則使用 2D 簡化解

        回傳:
            np.array([x, y, z])
        """
        anchors = self.anchors
        d = np.array(distances, dtype=float)
        n = len(anchors)

        if known_z is not None:
            return self._solve_known_z(anchors, d, known_z)

        if n < 4:
            raise ValueError("3D 定位需要至少 4 個 anchors。")

        # === 3D 線性化 ===
        A = 2 * (anchors[1:] - anchors[0])
        b = (
            np.sum(anchors[1:]**2, axis=1)
            - np.sum(anchors[0]**2)
            + d[0]**2
            - d[1:]**2
        )

        # === 最小平方解 ===
        pos = np.linalg.lstsq(A, b, rcond=None)[0]
        return pos

    # ------------------------------------------------------------
    def _solve_known_z(self, anchors, d, z_tag):
        """
        已知 Z (例如 Tag 高度固定）時的 2D 化簡求解。
        """
        n = len(anchors)
        if n < 3:
            raise ValueError("已知 Z 的情況下需要至少 3 個 anchors。")

        # 修正距離: 扣除垂直高度差
        dz = z_tag - anchors[:, 2]
        d_xy = np.sqrt(np.maximum(d**2 - dz**2, 0))

        # 以第一個 anchor 為基準建立線性方程
        anchors_2d = anchors[:, :2]
        A = 2 * (anchors_2d[1:] - anchors_2d[0])
        b = (
            np.sum(anchors_2d[1:]**2, axis=1)
            - np.sum(anchors_2d[0]**2)
            + d_xy[0]**2
            - d_xy[1:]**2
        )

        pos_2d = np.linalg.lstsq(A, b, rcond=None)[0]
        return np.array([pos_2d[0], pos_2d[1], z_tag])

    # ------------------------------------------------------------
    @staticmethod
    def test_solver():
        """
        測試與驗證公式正確性。
        """
        anchors = np.array([
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        true_pos = np.array([0.5, 0.5, 0.5])
        d = np.linalg.norm(anchors - true_pos, axis=1)

        solver = TrilaterationSolver3D(anchors)
        est = solver.solve(d)
        print("✅ 測試 (3D)")
        print("真實位置:", true_pos)
        print("估計結果:", np.round(est, 3))
        print("誤差:", np.linalg.norm(est - true_pos))

        # 測試已知 Z 模式
        z_known = 0.5
        est2 = solver.solve(d, known_z=z_known)
        print("\n✅ 測試 (已知 Z)")
        print("估計結果:", np.round(est2, 3))
        print("誤差:", np.linalg.norm(est2 - true_pos))







if __name__ == "__main__":
    import time
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    
    
    from SerialWorker import SerialWorker
    from UWBController import UWBController
    from UWBKalmanFilter import UWBKalmanFilter
    

    # === 初始化繪圖 ===
    def setup_plot(anchors):
        fig, ax = plt.subplots()
        ax.set_title("UWB 2D Positioning (Top View)")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.grid(True)
        ax.set_aspect('equal', 'box')

        anchors_np = np.array(anchors)
        ax.scatter(anchors_np[:, 0], anchors_np[:, 1],
                c='red', marker='^', s=100, label='Anchors')
        for i, (x, y, z) in enumerate(anchors):
            ax.text(x + 0.05, y + 0.05, f"A{i}", color='red')

        tag_dot, = ax.plot([], [], 'bo', markersize=10, label='Tag')
        path_line, = ax.plot([], [], 'b--', alpha=0.4)
        pos_text = ax.text(0.02, 0.95, "", transform=ax.transAxes)

        ax.legend()
        plt.tight_layout()

        # 儲存軌跡資料
        data = {"path_x": [], "path_y": [], "pos": [0, 0, 0]}

        # === 更新函數 ===
        def update(frame):
            x, y, z = data["pos"]
            data["path_x"].append(x)
            data["path_y"].append(y)
            if len(data["path_x"]) > 10:
                data["path_x"].pop(0)
                data["path_y"].pop(0)

            tag_dot.set_data([x], [y])
            path_line.set_data(data["path_x"], data["path_y"])
            pos_text.set_text(f"x={x:.2f}, y={y:.2f}, z={z:.2f}")
            return tag_dot, path_line, pos_text

        ani = FuncAnimation(fig, update, interval=50, blit=False, save_count=100)
        plt.show(block=False)

        return data, fig, ani

    anchor_ids = [0xFF00, 0xFF01, 0xFF02, 0xFF03]
    tag_ids = [0x0000]
    
    anchors = [
        (0, 0, 1.53),
        (-0.35, 1.12, 2.23),
        (2.57, 1.61, 2.21),
        (2.57, -0.51, 2.24)
    ]

    COM = "COM18"
    worker = SerialWorker(port=COM, baudrate=115200, debug_print=False)
    worker.open()
    uwb = UWBController(serial_worker=worker, debug_print=False)
    
    kf = UWBKalmanFilter(dt=0.2, process_var=1e-3, measurement_var=1e-1)

    # 🔹 初始化繪圖
    data, fig, ani = setup_plot(anchors)
    
    try:
        while True:
            reports = uwb.trigger_multiple(tag_ids[0], anchor_ids)
            dist_list = [r.distance_m if r else -1 for r in reports]
            rssi_list = [r.rssi_dbm if r else -1 for r in reports]
            
            solver = TrilaterationSolver3D(anchors)
            pos = solver.solve(dist_list, known_z=1.0)
            pos_filtered = kf.filter(pos)
            
            print(f"\t計算結果: [{pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}]", end="")
            print(f"📡 Tag {tag_ids[0]}: RSSI={rssi_list}, Dist={dist_list}  ")
            
                        
            # 更新畫面用的資料
            data["pos"] = pos_filtered
            plt.pause(0.05)

            # print(f"📍[{pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}] | RSSI={rssi_list} | Dist={dist_list}")

            
    except KeyboardInterrupt:
        print("Exiting...")

    worker.close()



