# uwb-positioning-system

## 專案總覽
這個儀專案整合了 **ESP32+DWM1000 韌體**、**對應硬體設計** 與 **Python 可視化 Host App**，目標是建置可以在室內部署的 UWB 定位系統。韌體負責超寬頻量測與序列埠通訊、硬體提供可量產的電路與機構、Host App 則把距離資料轉為即時 2D/3D 位置並提供後續交接時最重要的溝通介面。

## 目錄導覽
- `firmware/`：PlatformIO 專案，產生可燒錄到 ESP32-DWM1000 節點的韌體。
- `hardware/`：ESP32_DWM1000_V1 的 PCB、BOM、殼體與 3D 檔案。
- `host_app/`：Python + PyQt6 介面，透過序列埠操控節點、做距離採樣與定位演算法。

---

## Firmware（`firmware/ESP32-DWM1000`）
1. **建置環境**
   - 安裝 [PlatformIO CLI](https://platformio.org/) 或 VS Code + PlatformIO IDE。
   - 需要 ESP32 USB 驅動與 115200 bps 序列埠。
2. **編譯與燒錄**
   - `cd firmware/ESP32-DWM1000`
   - 編譯：`pio run`
   - 燒錄：`pio run -t upload`（可用 `-t monitor` 觀察訊息）
3. **節點角色**
   - `uwb_node_id` 由程式內部邏輯或使用者命令設定，Anchor 建議使用 `0xFF00` 起跳，Tag 使用 `0x0000` 起跳。
4. **主控指令**
   - 序列埠接受 ASCII 指令，換行分隔：
     - `ping <node_id>`：請 <node_id> 回報系統狀態與電壓。
     - `trigger <initiator_id> <responder_id>`：觸發 initiator 與 responder 間的 TWR 量測。
   - 回傳為單行 JSON，事件種類：
     - `{"event":"ping_resp","node_id":...,"system_state":...,"voltage_mv":...}`
     - `{"event":"range_report","node_a_id":...,"node_b_id":...,"distance_m":...,"rssi_dbm":...}`
     - `{"event":"range_final", ...}`（最終距離結果）。

---

## Hardware（`hardware/esp32_DWM1000_V1`）
1. **PCB 製造**
   - `pcb/Gerber_esp32_DWM1000_V1_2025-10-20.zip` 交給板廠即可生產。
   - `pcb/BOM_esp32_DWM1000_*.xlsx` 提供物料代號與封裝，搭配 `InteractiveBOM_*.html` 可快速查料。
   - `pcb/SCH_Schematic1_*.pdf` 與 `3D_*.step` 提供電路與 3D 參考。
2. **外殼/治具**
   - `shell/外殼.stl`、`shell/ESP32_DWM1000_外殼2 v26.f3z` 可 3D 列印或交由 Fusion 360 編修。
   - `shell/側按鍵*.stl` 與 `shell/壓克力.dxf` 用於按鍵、壓克力面板。

---

## Host App（`host_app/`）
Host App，負責和裝置溝通、資料視覺化與定位演算。

### 環境設定
1. 使用 Python 3.11（建議建立虛擬環境 `python -m venv venv` 並啟用）。
2. 安裝套件：`pip install -r requirements.txt`。
3. 依需求編輯 `config.json`：
   - `anchors`: `id`（與韌體相同的 16-bit ID）、`pos`（[x,y,z]，單位公尺）、`enable`。
   - `tags`: `id` 與 `z`、`enable`。Z 可固定高度以減化三角定位。

### 執行與操作
1. `python UWB_DEMO_APP.py` 開啟 PyQt6 介面。
2. 於左下輸入序列埠（例如 `COM7`）與鮑率（預設 115200），按「Connect」即可建立 `SerialWorker` 執行緒。
3. 程式啟動時計時器每 `update_position` 週期會：
   - 讀取 `config.json` 中啟用的 Tag、Anchor 列表。
   - 透過 `UWBController.trigger_multiple(tag_id, anchor_ids)` 走訪所有 anchor。
   - 每次 `trigger` 會送出 `trigger <tag> <anchor>`，等待 JSON 的 `range_report/range_final`。
   - 同時可使用 `ping_single`/`ping_all` 按鈕，送出 `ping <node>` 取得電壓與狀態。
4. `TrilaterationSolver3D` 會把有效距離組合轉為 (x,y,z)，再由 `UWBKalmanFilter` 平滑並顯示在平面圖與 RSSI 表格。
5. 介面支援：
   - 拖曳 Anchor 位置（`DraggableScatterPlotItem`），並可儲存為新的 `config.json`。
   - 清除 Tag 路徑、顯示即時 RSSI/距離列表、手動載入/儲存多組配置。
