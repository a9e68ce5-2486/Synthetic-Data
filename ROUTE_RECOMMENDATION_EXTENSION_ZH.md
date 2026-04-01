# 路線建議層擴充規劃

> 文件建立日期：2026-03-23
>  
> 最近更新日期：2026-03-24
>  
> 本文件對應 single-agent DRQN 主線完成後，如何把模型輸出轉成企業可直接採用的避難路線建議

## 一、為什麼需要這一層

目前 DRQN 已經能在：

- easy-stage
- blocked-stage
- multi-seed evaluation
- baseline comparison

上展現穩定且高效的 shelter-routing 能力。

但對企業而言，單純知道：

- reached rate 很高
- exposure 很低
- 某個 policy 比 heuristic 好

還不夠。

企業真正會問的是：

- 某棟樓的人應該往哪一個 shelter？
- 具體要走哪一條路？
- 如果主路封住，備援路線是什麼？
- 行人、車輛、shuttle 是否應該分流？

因此，接下來最重要的不是重做 DRQN，而是補一層 **route recommendation / route explanation layer**。

這一層的目的是把：

- step-by-step 的 graph policy

轉成：

- 人可以理解
- 管理端可以採用
- 能放進 SOP / dashboard / drill plan

的避難路線建議。

## 二、這一層要解決的核心問題

### 1. 從 policy 到完整路徑

目前 DRQN 內部做的是：

- 在每個 graph node 上選下一步 action

但企業需要的是：

- 一條完整可讀的 evacuation route

所以需要把：

- node-by-node decision trace

整理成：

- recommended path
- backup path
- target shelter

### 2. 從 graph node 到人類可讀資訊

目前內部表示方式主要是：

- OSM node id
- edge
- graph path

這對研究是夠的，但對企業使用者不夠。

需要補的輸出形式包括：

- 路口序列
- 道路名稱
- 關鍵轉折點
- 起點區域到 shelter 的文字化說明
- 地圖上的高亮路線

### 3. 從單一 agent 路徑到分區建議

企業通常不會逐人下達動態 action，而是需要：

- building-level
- zone-level
- department-level

的疏散建議。

所以模型輸出需要再往上抽象成：

- 哪一區的人優先走哪個 shelter
- 哪些路段應避免
- 哪些路段應作為備援通道

## 三、建議的功能拆分

### 第一階段：單一路線抽取

先做最小可用功能：

- 輸入：
  - 起點 node
  - 災害狀態
  - DRQN checkpoint
- 輸出：
  - recommended shelter
  - recommended path
  - path length
  - estimated exposure
  - estimated steps

這一階段先不要碰 UI，只要能從模型穩定抽出完整路徑即可。

目前狀態：

- 已完成第一版 prototype
- 已新增：
  - `route_recommendation.py`
  - `run_route_recommendation.sh`

目前已能輸出：

- `recommended shelter`
- `complete node path`
- `traversed edges`
- `steps`
- `exposure`
- `replan_count`
- `target_history`

### 第二階段：備援路線

在主路線之外，再輸出：

- backup route 1
- backup route 2

用途：

- 如果主路段 blocked
- 或某 bottleneck 過載

就能切換到備援方案。

目前狀態：

- 單一 agent 的明確多備援候選尚未完成
- 但 `zone-level` primary / backup route 第一版已完成
- 已新增：
  - `zone_assignment.py`
  - `run_zone_assignment.sh`
  - `zone_route_recommendation.py`
  - `run_zone_route_recommendation.sh`

目前已能針對每個 zone 輸出：

- `primary shelter`
- `backup shelter`
- `primary route`
- `backup route`

也就是說，備援路線能力已先在 **zone-level planning** 上落地。

### 第三階段：分區疏散建議

將多個起點聚成區域，例如：

- 行政樓
- 教學樓
- 宿舍區
- 停車場

輸出每個區域的：

- primary shelter
- primary route
- backup route

這會更接近企業真正的應用方式。

目前狀態：

- 已完成第一版 prototype
- 已能輸出：
  - `representative_start_node`
  - `primary_shelter`
  - `backup_shelter`
  - `primary_route`
  - `backup_route`

後續補強：

- 已加入 `route-feasibility filter`
- 會先保留 assignment 的原始結果，再用 route rollout 過濾出可達的最終 primary / backup shelter
- 已加入 `hard capacity-respecting assignment`
  - 會顯式輸出：
    - `primary_assigned_demand`
    - `backup_assigned_demand`
    - `overflow_demand`
    - `unassigned_demand`
- 已加入 `backup quality threshold`
  - 門檻為：
    - `EVAC_ZONE_BACKUP_MAX_STEP_RATIO = 1.50`
    - `EVAC_ZONE_BACKUP_MAX_EXPOSURE_RATIO = 2.50`
- 已加入 `primary route-quality-aware recommendation`
  - recommendation layer 會另外輸出：
    - `recommended_primary_shelter`
    - `recommended_backup_shelter`
    - `recommended_primary_route`
    - `recommended_backup_route`

新增輸出：

- `assigned_primary_shelter`
- `assigned_backup_shelter`
- `route_selection_mode`
- `feasible_route_count`
- `candidate_shelters`
- `backup_quality_ok`
- `recommended_backup_quality_ok`
- `recommended_primary_route_quality_score`
- `recommended_backup_route_quality_score`
- `backup_status`
- `recommended_backup_status`
- `backup_weak`
- `zones_with_weak_backup`
- `*_management_summary.txt`

目前限制：

- zone 仍是空間分群，不是真實 building / department 區塊
- 雖然已新增 `management summary`，但目前仍是文字化摘要，尚未接到 dashboard / map UI
- `capacity-enabled` scenario 已可形成合法且可達的 zone-level primary / backup recommendation
- 但 `recommended_*` 目前仍屬 recommendation layer，尚未直接回寫到 assignment core

### 第四階段：可視化輸出

將上述結果轉成：

- markdown report
- 地圖路線圖
- 管理端 dashboard 資料格式

這一層完成後，模型才真正變成可交付的決策支援工具。

## 四、技術上要怎麼做

### 1. route extraction

利用目前已存在的：

- DRQN controller
- current target
- candidate neighbors
- blocked-aware replanning

從起點開始 rollout，直到：

- 抵達 shelter
- 或達到 step limit

記錄整段：

- visited nodes
- traversed edges
- 是否觸發 replanning
- exposure accumulation

最後輸出成完整 path。

### 2. path post-processing

對 raw node path 做整理：

- 去掉重複震盪段
- 合併連續直行段
- 找出關鍵轉折點
- 標註 bottleneck edges

### 3. human-readable translation

若 road metadata 可得，則將 path 轉成：

- road names
- intersection sequence
- turn instructions

若 metadata 不完整，至少輸出：

- 起點節點
- 關鍵 checkpoint
- shelter 節點
- 路線示意圖

### 4. route confidence / quality indicators

每條建議路線應附帶品質資訊，例如：

- estimated reached probability
- estimated exposure
- estimated completion steps
- whether replanning was needed
- bottleneck risk level

這樣企業才不只是拿到一條線，而是能評估這條建議的可信度與風險。

## 五、企業端的實際用途

這一層完成後，模型可以支援企業做：

### 1. 個人或區域避難路線建議

例如：

- A 棟東側人員前往 Shelter 3
- 建議路線為 Route A
- 若東側道路 blocked，改走 Route B

### 2. SOP 與 drill planning

可將模型輸出轉成：

- 疏散 SOP
- drill plan
- emergency playbook

### 3. 瓶頸與資源配置分析

透過推薦路線與 bottleneck 資訊，企業可以判斷：

- 哪些路段應優先清障
- 哪些 shelter 應擴容
- 哪些地點應增設接駁點

## 六、與後續 multimodal / vehicle-aware extension 的關係

路線建議層不是最終版本，而是下一階段更複雜決策系統的基礎。

之後若做：

- vehicle-aware routing
- shelter-aware decision
- multimodal evacuation

這一層仍然可以沿用，只需要把輸出從：

- walk-only route

擴成：

- walk route
- car route
- shuttle-assisted route
- mixed-mode route

也就是說，現在先把 route recommendation layer 做好，不會浪費，反而是後續所有企業化輸出的基礎。

## 七、建議的實作順序

### Step 1

先做一個最小版本：

- 給定起點
- 輸出 DRQN 推薦 shelter 與完整 node path

目前狀態：

- 已完成

### Step 2

補上：

- path summary
- estimated exposure
- estimated steps
- replanning trace

目前狀態：

- 部分完成
- `estimated exposure`、`estimated steps`、`replanning trace` 已可輸出
- 尚未補完整的 path summary 與人類可讀說明

### Step 3

加入：

- backup route
- bottleneck annotation

### Step 4

再往：

- zone-level recommendation
- map visualization
- management report integration

推進。

## 八、目前建議

目前最合理的下一步不是直接做完整多代理互動，而是：

- 先保留現有 single-agent DRQN 主線
- 補 route recommendation layer

因為這會直接把目前的研究成果轉成企業真正能理解與採用的形式。

一句話總結：

- 你現在已經有「會決策的模型」
- 下一步要補的是「把決策結果轉成企業可採用的避難路線建議」
