# Host App 指南

## 檔案對照
| 檔案 | 功能摘要 |
| --- | --- |
| `SerialWorker.py` | 底層串列通訊執行緒，維持 `Serial` 連線、將裝置回傳的 JSON 事件塞入 queue，並提供 `send_command`/`read_response` API。 |
| `UWBController.py` | 封裝序列指令集合，包含 `ping`、`trigger`、`trigger_multiple` 等方法，並以資料類別 (`RangeResponse`, `PingResponse`) 回傳解析後結果。 |
| `TrilaterationSolver3D.py` | 以多個 Anchor 座標與距離解三點/多點定位的演算法，支援固定 Z 的 3D 求解並提供校正/排序工具。 |
| `UWBKalmanFilter.py` | 對定位結果進行單點 Kalman 濾波，降低量測噪訊與跳動。每個 Tag 會各自維持一個濾波器實例。 |
| `UWB_DEMO_APP.py` | PyQt6 主程式：控制介面、Anchor/Tag 管理、即時圖表、RSSI 表格、手動載入/儲存 `config.json`，並定期呼叫控制器量測距離。 |
| `config.json` | Anchor 與 Tag 的 ID、座標、啟用狀態與預設高度設定，GUI 讀取後即能還原場地配置。拖曳 Anchor 或儲存設定時也會覆寫這個檔案。 |
| `requirements.txt` | Host App 依賴的 Python 套件列表 (PyQt6、pyserial、pyqtgraph、numpy…)，交接時可直接 `pip install -r requirements.txt`。 |

# 🛰️ UWB 定位系統上位機 (PyQt6)

## 📋 系統概述
這是一個使用 **PyQt6 + pyqtgraph** 開發的上位機應用程式，用於顯示與控制 **UWB（超寬頻）定位系統**。  
支援多個 Anchor（基站）與 Tag（標籤）節點，能透過串口通訊進行距離量測、定位與地圖可視化操作。

---

## ⚙️ 功能總覽

### 🔌 串口連線控制
- 指定連線埠 (`COM`) 與鮑率 (`Baudrate`)。
- 支援：
  - **Connect**：連線到指定序列埠。
  - **Disconnect**：中斷連線。
  - **Ping**：測試節點電壓與回應狀態。

---

### 💾 設定管理（Save / Load）
- 可儲存與載入 **Anchor** 和 **Tag** 配置：
  - `Save`：儲存目前配置為 `config.json`。
  - `Load`：從檔案中載入設定。
- 啟動時自動讀取 `config.json`（若存在）。
- 設定檔內容包括：
  - Anchor ID、座標 (X/Y/Z)、啟用狀態。
  - Tag ID、高度 Z、啟用狀態。

---

### 📍 Anchor 管理
- 新增 / 移除 Anchor 節點。
- 可編輯每個 Anchor 的：
  - ID（16 進位顯示，如 `0xFF00`）
  - 座標 X / Y / Z。
- 支援啟用 / 停用。
- 可在地圖上**拖曳** Anchor 點直接修改位置（即時更新表格）。

---

### 🎯 Tag 管理
- 新增 / 移除 Tag。
- 可設定：
  - Tag ID。
  - 固定高度 Z。
- 支援啟用 / 停用。
- 顯示：
  - Tag 即時位置。
  - 移動軌跡（含濾波平滑）。

---

### 📶 RSSI 與距離顯示
- 以表格顯示：
  - Tag ID。
  - Anchor ID。
  - 距離 (m)。
  - RSSI (dBm)。
- 定期更新，顯示最新量測結果。

---

### 🔧 Anchor 自動校正 (Calibration)
- 啟用兩個以上的 Anchor 可進行自動校正。
- 自動測距所有 Anchor 配對。
- 使用 **多維尺度分析 (MDS)** 計算相對位置。
- 自動更新 Anchor 座標與地圖。
- 僅支持2D校正（Z軸固定,需將Anchor放置於同一平面）。

---

### 🗺️ 地圖繪製與互動
- 即時顯示：
  - Anchor（紅色三角形）。
  - Tag（彩色圓點）。
  - Tag 移動路徑。
- 使用 **pyqtgraph**：
  - 鎖定等比例座標。
  - 顯示格線與軸標籤 (X/Y)。
  - Anchor 點可拖曳以修改位置。

---

### 🧹 其他輔助功能
- `Clear Paths`：清除所有 Tag 的歷史軌跡。
- 狀態列顯示系統狀態、連線資訊與錯誤提示。
- 自動檢查 Anchor 數量不足時顯示警告。

---

## 🖥️ 操作介面說明
- 左側控制面板：
  - 系統控制、連線設定、Ping 測試、Anchor/Tag 管理、RSSI 監控與校正。
- 右側地圖區：
  - 即時顯示 Anchor、Tag、軌跡。
  - 可互動拖曳 Anchor 點設定座標。

---

## 🚀 執行方式

```bash
# 建立虛擬環境
python -m venv venv
source venv/bin/activate     # (Windows 用 venv\Scripts\activate)

# 安裝依賴
pip install -r requirements.txt

# 執行主程式
python UWB_DEMO_APP.py
```


