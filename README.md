## 🎯 研究所機器人學習與互動作業專案
此專案展示研究所 **機器人學習與互動 (Robot Learning and Interaction)** 課程的作業內容與結果。

---

## 📂 專案內容
- [Lab0](#lab0)：機器人行為編程暖身專案
- [Lab1](#lab1)：機器人定位 - 粒子濾波 (Particle Filter Localization)
- [Lab2](#lab2)：機器人導航與 SLAM
- [Lab3](#lab3)：機械手臂操控與物件抓取
- [Final Project](#final-project)：TurtleBot3 自駕車路標辨識

## 📌 Lab0：機器人行為編程暖身專案

📌 **目標**：熟悉 **ROS 環境**，並透過 **TurtleBot3 機器人** 在 Gazebo 模擬器中執行基本行為控制，如**行駛方形路徑 (Driving in a Square)** 和 **牆壁跟隨 (Wall Following)**。

### 🔹 主要步驟
1. **環境設置**
   - 安裝與設定 ROS。
   - 透過 Gazebo 模擬器運行 TurtleBot3。
2. **行駛方形路徑**
   - 讓機器人移動成方形軌跡，可使用時間控制或 **odometry (/odom topic)**。
   - 透過 **/cmd_vel** 控制機器人移動。
3. **牆壁跟隨**
   - 讓機器人沿著房間牆壁行駛。
   - 使用 **/scan (LiDAR 感測數據)** 來保持固定距離。

### 🔹 挑戰與解決方案
- **挑戰**：
  - ROS 指令與模擬環境的適應。
  - 感測器噪聲影響機器人行為。
- **解決方案**：
  - 使用 **RViz** 進行感測器數據可視化。
  - 優化控制參數，減少機器人行駛時的誤差。

### 🔹 成果展示
- 成功讓機器人在 Gazebo 模擬環境內執行 **行駛方形路徑** 和 **牆壁跟隨** 兩種行為。

---

## 📌 Lab1：機器人定位 - 粒子濾波 (Particle Filter Localization)

📌 **目標**：透過 **粒子濾波 (Particle Filter, Monte Carlo Localization, MCL)**，解決機器人定位問題，使機器人能夠根據感測器資訊估計自身位置。

### 🔹 主要步驟
1. **初始化粒子雲**：隨機生成機器人可能的位置分布。
2. **運動模型**：根據機器人的運動更新粒子狀態。
3. **測量模型**：根據感測數據更新粒子的權重。
4. **重新取樣**：根據權重重新抽樣粒子。
5. **融合噪聲**：考慮感測器與動作的不確定性。
6. **更新機器人位置估計**。
7. **參數調整與最佳化**。

### 🔹 挑戰與解決方案
- **挑戰**：感測器噪聲與環境不確定性會影響定位準確度。
- **解決方案**：
  - 使用高斯噪聲模擬不確定性，提升演算法穩定性。
  - 優化粒子數量與重新取樣機制，提高效率與準確性。

### 🔹 成果展示
- 透過 **RViz 可視化** 機器人定位過程。
- 成功收斂至機器人真實位置，提升定位準確度。

---

## 📌 Lab2：機器人導航與 SLAM

📌 **目標**：使用 **SLAM (Simultaneous Localization and Mapping)**，讓 TurtleBot3 能夠在未知環境中構建地圖並實現自主導航。

### 🔹 主要步驟
1. **建立 Gazebo 模擬環境**：設計自訂房間作為導航測試場景。
2. **使用 SLAM 建立地圖**：透過 LiDAR 感測器掃描環境，建立佔據柵格地圖。
3. **儲存與載入地圖**：確保機器人能夠在相同環境中重新使用先前建立的地圖。
4. **導航至目標點**：使用 **DWA (Dynamic Window Approach) 局部規劃**，讓機器人自動避障並移動至指定位置。

### 🔹 挑戰與解決方案
- **挑戰**：
  - 機器人導航時可能會迷失方向，影響準確性。
  - 障礙物較多時，DWA 規劃可能無法順利導航。
- **解決方案**：
  - 使用 **Adaptive Monte Carlo Localization (AMCL)** 來提升機器人定位準確度。
  - 設計更清晰的地圖，避免過於複雜的環境影響導航。

### 🔹 成果展示
- **SLAM 地圖生成成功**，機器人能夠在環境中導航並抵達目標位置。
- **錄製導航過程 GIF/MP4**，顯示 TurtleBot3 順利移動至指定位置。

影片網址:
https://drive.google.com/file/d/1i3nD4K4rrngkJCywVfXllx9LckXkWuUr/view?usp=sharing
https://drive.google.com/file/d/1x9EPuoEvxqhzUekzj3842dX5swIKTUXX/view?usp=drive_link

---

## 📌 Lab3：機械手臂操控與物件抓取

📌 **目標**：學習機械手臂運動控制，並實現機器人手臂的 **TrafficBot** 手勢與 **抓取物件** 的任務。

### 🔹 主要步驟
1. **控制手臂運動**：透過 MoveIt! 控制 TurtleBot3 OpenMANIPULATOR 手臂。
2. **TrafficBot 訊號動作**：依據輸入指令控制機械手臂模擬交通指揮。
3. **抓取物件**：控制機械手臂抓取地面上的物品。

### 🔹 挑戰與解決方案
- **挑戰**：手臂關節控制較為複雜，需要調整運動學參數。
- **解決方案**：透過 **MoveIt! GUI** 進行初步測試，再將參數應用於程式碼中。

### 🔹 成果展示
- **TrafficBot 模擬成功**，機械手臂根據指令擺動手臂。
- **成功抓取物件**，並在 Gazebo 中完成物件移動。

影片網址:
https://drive.google.com/file/d/1usE3HOgll0BWGjACb0kYz6-vZfjTTLNW/view?usp=drive_link
https://drive.google.com/file/d/1PWSTSvj1oml5-ocjz63HJksr2Hzd3smB/view?usp=drive_link

---

## 📌 Final Project：TurtleBot3 自駕車路標辨識

📌 **目標**：透過 **YOLOv5** 與 **ResNet18**，讓 TurtleBot3 **辨識路標與紅綠燈**，並依照辨識結果自動行駛。

### 🔹 方法與技術
1. **路標偵測 (YOLOv5)**：
   - 訓練 YOLOv5 模型來辨識 `停止 (stop)`, `速限 (speed limit)`, `人行道 (crosswalk)` 等路標。
   - 訓練數據集來自 Kaggle，格式轉換 XML → YOLO TXT 以適應訓練需求。
2. **紅綠燈顏色辨識 (ResNet18)**：
   - 使用 CNN 模型辨識 `紅 (停止)`、`黃 (減速)`、`綠 (前進)`。
   - 先使用 OpenCV 嘗試傳統方法，但成功率較低，最終改用 ResNet18 訓練模型。
3. **模擬環境與測試**：
   - 在 **Gazebo** 中建構模擬環境，將標誌與紅綠燈貼在物件上。
   - 透過 ROS 訂閱攝影機畫面，進行即時偵測並驅動 TurtleBot3 動作。

### 🔹 成果展示
- TurtleBot3 **成功辨識路標與紅綠燈**，並能根據結果做出正確行動。
- 在 Gazebo 模擬環境中，機器人能夠自動行駛並避開錯誤的判斷。

影片網址:
https://drive.google.com/file/d/1k3_hfl_hKrMkEj8RVbd1tC06Yo6DF_GU/view?usp=sharing

---

## 📌 結論
本專案涵蓋了 **機器人定位 (MCL)**、**導航與 SLAM**、**機械手臂操控** 及 **自駕車應用** 等多項人工智慧技術，透過實作加深對 AI 在機器人領域應用的理解。未來可進一步優化導航系統，並嘗試在實體機器人上測試自駕車功能。🚀

