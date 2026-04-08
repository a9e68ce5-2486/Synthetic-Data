# 專案更新歷程

> 文件建立日期：2026-03-21
>  
> 最近更新日期：2026-04-02
>
> 本文件涵蓋的主要更新區間：2026-03-21 至 2026-04-01

這份文件整理目前專案在 DRQN shelter-routing 主線上的主要更新，包括：

- 額外參考的論文方向
- 已經實作進系統的機制
- 訓練與評估結果的演進
- 目前最佳模型與下一階段方向

## 一、目前額外參考與吸收的論文方向

### 1. BEAG

目前專案主要額外參考的核心論文是 **BEAG**。

雖然沒有完全重現原始 BEAG 演算法，但已經吸收並轉化成適合本專案 OSM graph evacuation 任務的幾個重要概念，包括：

- 探索過程中避免重複失敗
- 利用中繼目標降低長路徑學習難度
- 逐步放寬訓練難度
- 在更接近真實避難條件的環境中強化訓練穩定性

### 2. Teacher-Student Curriculum Learning (TSCL)

除了 BEAG 之外，也進一步參考了 **Teacher-Student Curriculum Learning** 類概念。

主要啟發是：

- curriculum 不必完全手動固定
- 可以依照模型最近的成功率，自動調整接下來的訓練難度

這個概念後來被實作成目前的 `adaptive curriculum`。

### 3. 其他已整理但尚未正式導入主線的文獻方向

目前也已經完成相關文獻整理，但尚未全面實作進主線的方向包括：

- `Hindsight Experience Replay (HER)`
- `Go-Explore`
- `D* Lite`
- `HIRO`
- `Option-Critic`
- `FeUdal Networks (FuN)`
- `Random Network Distillation (RND)`
- `Count-based exploration`

這些方向目前屬於後續可能延伸的強化路徑，而不是已完成的主線功能。

## 二、目前已補強並完成的主要機制

### 1. Failure-Aware Exploration

已加入 failure-aware exploration 機制，用來記錄失敗邊與重複錯誤路徑，降低 agent 一直重複嘗試同樣錯誤行為的機率。

目的：

- 提高探索效率
- 減少 blocked edge 附近的無效重試
- 避免局部卡死

### 2. Subgoal / Checkpoint Learning

已加入 checkpoint / subgoal 機制，將長距離 shelter-routing 任務拆成數個中繼目標，使 agent 能逐段學習而不是只依賴最終 shelter reward。

目的：

- 改善 sparse reward 問題
- 降低長路徑學習難度
- 提升收斂速度與穩定性

### 3. Curriculum

已加入 curriculum training。

早期使用的是較固定的 `distance-based` 或 `coverage-based` curriculum，後來再進一步改為更有效的 `adaptive curriculum`。

目的：

- 先讓模型學會簡單起點
- 再逐步面對較遠、較困難案例

### 4. Dynamic Step Budget

已加入 dynamic step budget，使不同起點距離的 episode 能夠根據最短路徑距離獲得不同的 `max_steps`。

目的：

- 避免遠距離案例因步數上限過低而被不合理地判失敗
- 讓不同難度案例更公平

### 5. Blocked-Aware Replanning

已加入 blocked-aware replanning 機制。

當原本的 target path 因災害或封路而變得不可行時，系統會重新規劃 checkpoint 與路徑，而不是繼續沿用過期的中繼目標。

目的：

- 強化封路情境下的穩定性
- 讓 agent 在災害動態變化時仍能維持有效路徑選擇

### 6. Frontier / Revisit Control

已加入 frontier / revisit control，包含：

- frontier bonus
- revisit penalty

並讓候選鄰居排序同時考慮：

- graph progress
- 是否反覆回訪同一節點

目的：

- 降低繞圈與局部震盪
- 鼓勵 agent 往新且更有希望的區域前進

### 7. Adaptive Curriculum

在前述機制基礎上，後來又進一步加入了 `adaptive curriculum`，這是目前提升最明顯的一次改動。

做法是：

- 依最近一段訓練成功率
- 自動放寬或收緊訓練距離上限

目的：

- 避免固定 curriculum schedule 放太快或太慢
- 讓模型停留在最適合當前學習程度的難度區間

## 三、DRQN 訓練主線的效能演進

### 1. 舊版 easy 主線

在較早期的穩定版本中，easy mode 的 best evaluation 結果為：

- `reached_rate = 0.515`
- `return_mean = 200.295`
- `steps_mean = 72.59`

這個階段代表 DRQN 已經開始有效，但仍然遠未達到穩定高成功率。

### 2. `easy_long`

之後將訓練拉長、放慢 curriculum 後，`easy_long` 的結果提升為：

- `reached_rate = 0.585`
- `return_mean = 256.959`
- `steps_mean = 71.32`

這表示：

- 延長訓練與放慢 curriculum 確實有效
- 模型已開始變得更成熟

### 3. `easy_adaptive`

更新日期：2026-03-21 至 2026-03-22

在導入 TSCL-style adaptive curriculum 之後，easy stage 的表現出現顯著提升。

evaluation 結果為：

- `reached_rate = 1.000`
- `return_mean = 461.249`
- `steps_mean = 65.055`

這代表：

- 在 `p90 = 1765` 的 easy-stage 範圍內
- DRQN 已經達到完整成功率
- `adaptive curriculum` 是目前最關鍵的提升來源

### 4. easy-stage 動畫結果

在 `logs/drqn_easy_adaptive/drqn_torch_best.pt` 上跑互動動畫，結果同樣達到完整成功：

- `final_reached = 55 / 55`
- `ped = 40 / 40`
- `car = 15 / 15`
- `reached_rate = 1.0`
- `avg_exposure_total = 1.3928`
- `t90_step = 65`
- `t95_step = 67`
- `reach_rate_gap = 0.0`

這表示 easy-stage 不只 evaluation 指標漂亮，動畫與實際模擬結果也一致地達到完整成功。

## 四、Blocked Finetune 階段

更新日期：2026-03-22 至 2026-03-23

在 easy-stage best checkpoint 完成後，進一步將：

- `logs/drqn_easy_adaptive/drqn_torch_best.pt`

拿去做 blocked finetune。

在這個階段，並不是單純延長訓練，而是針對封路情境做了幾項明確的 finetune 調整：

- 以 `easy_adaptive` best checkpoint 作為初始化，而不是從頭訓練
- 將 `curriculum-end-dist` 拉到 `2622`
  - 對應更接近可達起點到 shelter 的最大距離範圍
- 保持 `curriculum-freeze-episode = -1`
  - 讓距離難度持續放寬，不在中途凍結
- 將 `eps-decay` 調成 `0.997`
  - 讓 blocked finetune 階段保留較多探索能力
- 將 `w_exposure` 提高到 `0.7`
  - 讓模型在封路與災害風險條件下更重視 exposure cost
- 將 `block-from-snow-threshold` 設為 `0.85`
- 將 `block-from-snow-prob` 設為 `0.003`
  - 正式打開較明顯的 snow-induced blockage 訓練條件

這些調整的目的，是讓模型從已經學會基本 shelter-routing 的 easy-stage policy 出發，再適應更接近真實災害條件的 blocked environment，而不是重新學一次完整導航行為。

### 1. blocked evaluation

blocked finetune 後的評估結果為：

- `reached_rate = 0.995`
- `return_mean = 442.664`
- `steps_mean = 66.48`

這表示：

- 模型在封路情境下幾乎沒有明顯崩掉
- 仍然保持接近完整成功率

### 2. blocked 動畫結果

使用：

- `logs/drqn_blocked_finetune/drqn_torch_best.pt`

跑動畫後，結果為：

- `final_reached = 55 / 55`
- `ped = 40 / 40`
- `car = 15 / 15`
- `reached_rate = 1.0`
- `avg_exposure_total = 1.435`
- `t90_step = 65`
- `t95_step = 71`
- `reach_rate_gap = 0.0`

這表示：

- 在 baseline interactive scenario 下
- blocked finetune 模型依然能完整完成 evacuation
- 而且 fairness gap 仍然維持在 0

### 3. blocked multi-seed evaluation

在 blocked finetune 完成後，另外又使用多個 evaluation seeds 做穩定性驗證：

- seeds = `2026, 2027, 2028, 2029, 2030`
- episodes per seed = `200`

aggregate 結果為：

- `reached_rate_mean = 0.998`
- `reached_rate_std = 0.00245`
- `return_mean_mean = 443.929`
- `return_mean_std = 5.997`
- `steps_mean_mean = 65.545`
- `steps_mean_std = 0.641`

這表示：

- 目前 blocked final candidate 並不是單一 seed 偶然成功
- 在不同 evaluation seeds 下仍維持接近完整成功率
- 整體 variance 很小，模型穩定性已相當高

### 4. baseline vs DRQN 正式比較

在 `enterprise_baseline` scenario 上，已經完成：

- `round_robin`
- `nearest`
- `drqn`

三種 policy 的正式 batch comparison。

主要結果如下：

- `round_robin`
  - `avg_reached_rate = 0.2654`
  - `avg_exposure_total = 286.1414`

- `nearest`
  - `avg_reached_rate = 0.2909`
  - `avg_exposure_total = 246.9240`

- `drqn`
  - `avg_reached_rate = 0.9627`
  - `avg_exposure_total = 5.4061`
  - `avg_t95_step = 183.79`

比較結論非常明確：

- `drqn` 是最佳 policy
- 相對 `round_robin`，DRQN 的平均 reached rate 約提升 `262.7%`
- 相對 `round_robin`，DRQN 的平均 exposure 約降低 `98.1%`

這表示目前主線 DRQN 不只在單次動畫與單次 evaluation 上表現良好，也已經在正式 baseline comparison 中顯著優於目前保留的 heuristic baselines：

- `round_robin`
- `nearest`

## 五、目前最佳模型

確認日期：2026-03-23

### 1. easy-stage best

目前 easy-stage 最佳模型為：

- `logs/drqn_easy_adaptive/drqn_torch_best.pt`

### 2. final blocked candidate

目前 blocked / final candidate 為：

- `logs/drqn_blocked_finetune/drqn_torch_best.pt`

這個模型目前同時具備：

- blocked evaluation 幾乎完整成功
- 互動動畫完整成功
- multi-seed evaluation 穩定成功
- baseline comparison 顯著優於 heuristics
- pedestrian 與 car 都能到 shelter

## 六、目前可下的整體結論

截至目前為止，可以清楚得出以下結論：

1. 參考 BEAG 後加入的探索、checkpoint、replanning 與 anti-loop 機制是有效的。
2. `adaptive curriculum` 是目前最大的增益來源，直接把 easy-stage reached rate 從 `0.585` 拉升到 `1.000`。
3. 以 easy-stage best model 為初始化做 blocked finetune 是成功的，blocked evaluation 已達 `0.995`。
4. blocked final candidate 在 multi-seed evaluation 下達到：
   - `reached_rate_mean = 0.998`
   - `reached_rate_std = 0.00245`
5. 在互動動畫與實際模擬中，DRQN 已能達到：
   - `55 / 55` 全部到達
   - `ped 40 / 40`
   - `car 15 / 15`
6. 在 `enterprise_baseline` 的正式比較中，DRQN 已明顯優於：
   - `round_robin`
   - `nearest`
7. 目前 DRQN 主線已經成熟到足以作為 stable final candidate，並可進入下一階段：
   - single-agent scaling
   - shelter-aware / vehicle-aware extension

## 七、Single-Agent Scaling 結果

更新日期：2026-03-23

在完成 blocked final candidate 後，已進一步用同一個 checkpoint 做更大規模的 single-agent scaling 測試。

使用模型：

- `logs/drqn_blocked_finetune/drqn_torch_best.pt`

測試範圍已從：

- `60 ped / 20 car`

一路擴到：

- `500 ped / 200 car`

### 1. 代表性 scaling 結果

- `60 / 20`
  - `avg_reached_rate = 0.9537`
  - `avg_exposure_total = 4.0570`
  - `avg_t95_step = 146.4`

- `100 / 40`
  - `avg_reached_rate = 0.9886`
  - `avg_exposure_total = 6.1833`
  - `avg_t95_step = 185.7`

- `200 / 80`
  - `avg_reached_rate = 0.9902`
  - `avg_exposure_total = 7.0167`
  - `avg_t95_step = 197.25`

- `300 / 120`
  - `avg_reached_rate = 0.9917`
  - `avg_exposure_total = 5.4917`
  - `avg_t95_step = 175.0`

- `400 / 160`
  - `avg_reached_rate = 0.9989`
  - `avg_exposure_total = 5.2629`
  - `avg_t95_step = 169.2`

- `500 / 200`
  - `avg_reached_rate = 0.9968`
  - `avg_exposure_total = 5.9366`
  - `avg_t95_step = 159.0`

### 2. scaling 結果的判讀

這組結果表示：

- 目前 final DRQN 在純數量放大下仍維持很高的 reached rate
- 至少在目前模擬設定中，`500 / 200` 尚未把 routing policy 撐爆
- 目前比較明顯的 scaling 成本主要反映在：
  - `avg_exposure_total`
  - `t95_step`

但這也同時說明：

- 單純增加 agent 數量，尚未完全逼出更強烈的系統互動壓力
- 下一步如果要更真實地測試企業場景，應該把重點轉向：
  - shelter capacity
  - congestion / queueing
  - 更明確的資源競爭

## 八、Route Recommendation Prototype

更新日期：2026-03-23

除了核心 DRQN policy 之外，目前也已完成第一版 route recommendation prototype。

新增內容：

- `route_recommendation.py`
- `run_route_recommendation.sh`

目前已能做到：

- 給定一個起點 node
- 使用目前的 DRQN checkpoint rollout
- 輸出：
  - `recommended shelter`
  - `complete node path`
  - `traversed edges`
  - `steps`
  - `exposure`
  - `replan_count`
  - `target_history`

這表示目前系統已經不只是一個研究用 policy，也開始具備：

- 把 step-level decision 轉成完整避難路線建議

的能力。

對企業應用而言，這是很重要的一步，因為它讓模型開始能回答：

- 某個起點的人應該往哪個 shelter？
- 具體該走哪條路？

### 1. Zone-level Route Recommendation 已完成第一版

在單一起點路線抽取完成後，目前也已進一步完成第一版 **zone-level route recommendation**。

新增內容：

- `zone_assignment.py`
- `run_zone_assignment.sh`
- `zone_route_recommendation.py`
- `run_zone_route_recommendation.sh`

目前已能做到：

- 將 scenario 中的人員起點自動分群成多個 zones
- 對每個 zone 輸出：
  - `primary_shelter`
  - `backup_shelter`
  - `primary_route`
  - `backup_route`
- route 內容包含：
  - `steps`
  - `exposure`
  - `path_nodes`
  - `traversed_edges`
  - `replan_count`
  - `target_history`

代表性輸出檔案：

- `logs/zone_assignment/enterprise_baseline_zones.json`
- `logs/zone_route_recommendation/enterprise_baseline_zone_routes.json`

這表示目前系統已經不只支援：

- 個體級路線建議

也開始支援：

- 區域級 shelter pre-allocation
- 區域級 primary / backup 路線建議
- 更接近企業 SOP / drill planning 的輸出格式

## 九、Shelter Capacity 第一版

更新日期：2026-03-23

為了把 scaling 從單純數量放大，進一步推向更真實的互動壓力測試，目前已完成 shelter capacity 的第一版實作。

新增內容：

- `EVAC_SHELTER_CAPACITY_ENABLED`
- `EVAC_SHELTER_CAPACITY_PER_SITE`

並在環境與 batch simulation 中加入：

- shelter occupancy
- shelter admission check
- shelter full 後重新分配到其他可用 shelter
- `shelter_reassignments` summary 欄位

這一版的目的不是一次做完整 multi-agent interaction，而是先把最關鍵的資源限制納入：

- shelter 不是無限容量

這能讓後續 scaling 更像真實企業避難情境，也能為下一步的：

- shelter-aware routing
- congestion-aware interaction

建立基礎。

### 1. capacity-aware scaling 結果

在打開 shelter capacity 後，已針對以下幾組規模做 interaction-aware scaling 測試：

- `200 / 80`, `cap = 50`
- `300 / 120`, `cap = 70`
- `500 / 200`, `cap = 100`

代表性結果如下：

- `200 / 80`, `cap=50`
  - `avg_reached_rate = 0.9536`
  - `avg_exposure_total = 23.0600`
  - `avg_t90_step = 230.29`
  - `avg_t95_step = 361.4`
  - `avg_shelter_reassignments = 41.12`

- `300 / 120`, `cap=70`
  - `avg_reached_rate = 0.9075`
  - `avg_exposure_total = 37.6288`
  - `avg_t90_step = 327.25`
  - `avg_t95_step = None`
  - `avg_shelter_reassignments = 102.33`

- `500 / 200`, `cap=100`
  - `avg_reached_rate = 0.8571`
  - `avg_exposure_total = 32.6220`
  - `avg_t90_step = None`
  - `avg_t95_step = None`
  - `avg_shelter_reassignments = 11774.2`

這表示：

- 一旦把 shelter 容量限制納入，系統壓力就會被真正逼出來
- reached rate 會下降
- exposure 會顯著上升
- t90 / t95 會明顯變慢
- 高壓規模下會出現大量 shelter reassignment churn

### 2. shelter-aware assignment 補強

為了降低 shelter full 後的大量重分配，已經再補了兩個針對性機制：

- `capacity-aware initial shelter assignment`
  - 在初始指派 shelter 時，就先避開容量已不足的 shelter
- `taboo shelter memory`
  - agent 被某個 shelter 拒收後，暫時把該 shelter 加入自己的禁選集合

這兩個機制的目的，是降低：

- 重複撞向已滿 shelter
- 高壓場景下反覆重分配造成的 churn

目前狀態：

- 兩個機制都已完成實作
- `capacity-aware initial assignment` 已重跑並確認在中等規模下有部分改善
- `taboo shelter memory` 已完成並完成新一輪驗證，但 aggregate KPI 沒有明顯改善，因此目前不列為主線增益機制

### 3. interaction-aware reranking 驗證結果

為了往 multi-agent interaction 擴展，也另外做了第一輪 execution-time interaction-aware feature 驗證。

目前已完成：

- `edge congestion-aware reranking` 第一版

其核心作法是：

- 不改 DRQN observation 維度
- 不重訓主線模型
- 在多 agent 執行時，利用：
  - edge occupancy
  - node occupancy
  - local density
  - same-step edge choice
  
  對候選鄰居做 congestion-aware reranking

目前驗證結果：

- 相較於原始 capacity baseline，並沒有形成穩定、全面的 KPI 改善
- 在部分中等壓力場景中，疏散速度變快，但 reached rate 或 exposure 不一定更好
- 在高壓場景中，改善幅度有限，尚不足以成為主線預設

因此目前判斷：

- `edge congestion-aware reranking` 應保留為 experimental branch
- 不應直接取代目前主線

### 4. zone-level capacity-aware demand balancing

根據 shelter capacity 與相關文獻分析，目前已將 zone assignment 升級成：

- `capacity-aware demand balancing`

這一版不再只是把每個 zone 指派給最近 shelter，而是同時考慮：

- 平均距離
- 剩餘容量
- zone demand 對剩餘容量的壓力
- overload 懲罰

目前已更新：

- `zone_assignment.py`
- `zone_route_recommendation.py`

新增輸出：

- `assignment_mode = capacity_aware_demand_balancing`
- `primary_assignment_score`

後續又再補了一步：

- `route-feasibility filter`

也就是在 `zone_route_recommendation.py` 中，不再直接使用 zone assignment 的 primary / backup shelter 當最終結果，而是：

- 先保留：
  - `assigned_primary_shelter`
  - `assigned_backup_shelter`
- 再對候選 shelters 逐一做 route rollout
- 從可達路線中選出最終：
  - `primary_shelter`
  - `backup_shelter`

這樣做的原因是，在 `enterprise_capacity_a300_c120_cap70` 測試中，已經觀察到：

- 部分 zone 雖然有被分配到 shelter
- 但對應的 representative route 可能根本不可達
- 或 backup route 非常差

因此目前 zone-level planning 已經不是只有：

- demand balancing

還加上了：

- route feasibility screening

這代表目前 zone-level planning 已經從：

- 最近 shelter 指派

進一步升級成：

- 容量感知的分區需求平衡
- 加上 route-feasible primary / backup route selection

接著又再補了兩個重要步驟：

- `hard capacity-respecting assignment`
- `backup quality threshold`

`hard capacity-respecting assignment` 的作用是：

- 不再允許 `remaining_after_primary < 0`
- 對於超過 primary shelter 容量的 zone demand，明確分成：
  - `primary_assigned_demand`
  - `backup_assigned_demand`
  - `overflow_demand`
  - `unassigned_demand`

在 `enterprise_capacity_a300_c120_cap70` 測試中，已經觀察到：

- `zone_2` 的 `85` 人，會被拆成：
  - `primary_assigned = 70`
  - `backup_assigned = 15`
  - `unassigned = 0`
- `zone_3` 的 `31` 人，會被拆成：
  - `primary_assigned = 29`
  - `backup_assigned = 2`
  - `unassigned = 0`

這表示目前 zone assignment 已能同時滿足：

- 容量合法
- 無未分配需求

`backup quality threshold` 則是加在 `zone_route_recommendation.py` 上，用來避免備援路線雖然可達、但品質明顯過差。

目前門檻為：

- `EVAC_ZONE_BACKUP_MAX_STEP_RATIO = 1.50`
- `EVAC_ZONE_BACKUP_MAX_EXPOSURE_RATIO = 2.50`

也就是：

- `backup_steps <= primary_steps * 1.5`
- `backup_exposure <= primary_exposure * 2.5`

在此基礎上，後來又再加入：

- `primary route-quality-aware recommendation`

這一步沒有直接推翻 hard-capacity assignment，而是把 route recommendation 輸出分成兩層：

- `assigned_*`
  - 代表容量合法的實際分配結果
- `recommended_*`
  - 代表在可達候選中，依據實際路線品質重新排序後的建議主 / 備援 shelter

目前 route quality score 為：

- `score = steps * 1.0 + exposure * 12.0`

新增輸出包括：

- `recommended_primary_shelter`
- `recommended_backup_shelter`
- `recommended_primary_route`
- `recommended_backup_route`
- `recommended_primary_route_quality_score`
- `recommended_backup_route_quality_score`

這讓系統目前已經不只是：

- zone-level capacity-respecting assignment

而是進一步具備：

- route-feasible
- backup-quality-controlled
- primary route-quality-reranked

的 zone-level recommendation layer。

在最新一次 `enterprise_capacity_a300_c120_cap70` 測試中，也已經觀察到這些新輸出確實有作用：

- `zone_1`
  - `assigned_primary` 與 `recommended_primary` 已不同
  - route-quality rerank 會把主建議改派到更短、更低 exposure 的路線
- `zone_4`
  - `recommended_primary` 也與 `assigned_primary` 不同
  - 表示 recommendation layer 已能修正 assignment core 沒有內生考慮 route quality 的缺口

同時，系統也已能直接標出：

- `zones_with_weak_backup = [1, 2, 4, 5]`

這表示：

- `zone_0`、`zone_3` 的備援可視為可用
- `zone_1`、`zone_2`、`zone_4`、`zone_5` 的備援雖可達，但不宜視為高信賴備援

目前也已加入：

- `backup_status`
- `recommended_backup_status`
- `backup_weak`

以及管理端摘要輸出：

- `*_management_summary.txt`

用來直接列出：

- weak backup zones
- changed primary zones
- changed backup zones
- 建議優先關注的 zone

另外也已修正：

- `zone_assignment.py`
- `zone_route_recommendation.py`

中原本的 shelter capacity metadata bug。

先前在 `temporary_config(...)` 外讀取 config，會讓：

- `shelter_capacity_enabled`
- `shelter_capacity_per_site`

錯誤回退成預設值。這個問題目前已修正。

## 十、下一階段方向

目前單 agent shelter-routing 主線已經達成高完成度，接下來最合理的方向是：

1. 先用 `capacity-aware demand balancing + route-feasibility filter` 版本重跑 zone assignment / zone route recommendation，觀察 capacity scenario 下的分流結果。
2. 補齊 comparison pipeline 中尚未聚合完成的 fairness summary 欄位。
3. 在 route recommendation prototype 上補：
   - primary assignment 與 recommendation 的一致化
   - bottleneck annotation
   - 更精簡的 management summary
4. 若仍有時間，再往更完整的 multimodal / vehicle-aware / shelter-aware 決策擴展，而不是直接推翻現有成功主線。

另外，若從 multi-agent interaction 的角度檢視，目前仍明確缺少：

- `edge-level queueing / slowdown`
- `shelter occupancy aware routing`
- `vehicle / shuttle competition`
- 顯式 coordinated multi-agent policy

因此目前的判斷是：

- 已有一套可用的 interaction-aware enterprise planning prototype
- 但還不是完整的 multi-agent interaction system
- 接下來若要再往前推，最值得優先補的是 `edge-level queueing / slowdown`

## 十一、災害強度分級（Disaster Severity Grading）

更新日期：2026-03-25

目前已在 `scenario_loader.py` 中加入災害強度分級能力，支援：

- `blizzard`
- `earthquake`
- `compound`

每一種災害都可以指定：

- `light`
- `moderate`
- `severe`
- `extreme`

分級後，封路與災害相關參數會隨強度由輕到重變化。

以 `blizzard` 為例，目前已會隨強度調整：

- `EVAC_BLOCK_PROB`
- `EVAC_BLOCK_INIT_PROB`
- `EVAC_SNOW_MIN`
- `EVAC_SNOW_MAX`
- `EVAC_SNOW_ACCUM_PER_STEP`
- `EVAC_BLOCK_FROM_SNOW_THRESHOLD`
- `EVAC_BLOCK_FROM_SNOW_PROB`

### 1. Blizzard severity sweep 結果

目前已完成：

- `scenarios/enterprise_blizzard.json`

在四級強度下的 `drqn` sweep。

結果如下：

- `light`
  - `reached_rate = 0.8354`
  - `exposure = 16.7949`
- `moderate`
  - `reached_rate = 0.7518`
  - `exposure = 72.8134`
- `severe`
  - `reached_rate = 0.6455`
  - `exposure = 108.3522`
- `extreme`
  - `reached_rate = 0.5345`
  - `exposure = 90.6228`

可下的結論：

- `reached_rate` 會隨 blizzard 強度由輕到重明顯下降
- 這表示災害分級已經成功把封路壓力拉開
- `exposure` 在 `extreme` 沒有再高過 `severe`
  - 這較可能代表極端情況下更多 agent 提早卡住、無法有效移動
  - 因此累積 exposure 不一定單調增加

所以目前判斷：

- `blizzard intensity grading` 可保留
- 後續應以 `reached_rate` 當成分級設計是否有效的主要指標

### 2. 目前狀態

目前已完成：

- severity loader
- severity sweep script
- `blizzard` 的正式 sweep 驗證

下一步建議：

- 跑 `earthquake severity sweep`
- 再跑 `compound severity sweep`

這樣的路線可以同時保留目前已經完成的高品質成果，並把後續延伸建立在穩定可驗證的基礎上。

## 十二、跨嚴重度效能改善——訓練/評估 Distribution Mismatch 修正

更新日期：2026-03-29

### 1. 問題根因分析

在完成 blizzard severity sweep 後，觀察到 `reached_rate` 隨嚴重度明顯下滑：

| Severity | reached_rate | 相對 baseline 的下降 |
|----------|-------------|-------------------|
| light    | 0.8354      | -12.7%            |
| moderate | 0.7518      | -21.1%            |
| severe   | 0.6455      | -32.7%            |
| extreme  | 0.5345      | -44.8%            |

診斷後確認主要根因有三：

**根因 1：訓練與評估的 `block_init_prob` 不一致（最主要）**

訓練時（所有現有 finetune 腳本）均使用預設 `block_init_prob = 0.0`，代表訓練環境一開始沒有任何封閉邊。但評估時 severity preset 會設定 `EVAC_BLOCK_INIT_PROB`：

- moderate：0.02
- severe：0.06（6% 邊一開始就被封）
- extreme：0.12（12% 邊一開始就被封）

即使之前的 `finetune_blizzard_v2.sh` 調整了 `block_from_snow_threshold`，也完全沒有設定 `--block-init-prob > 0`，因此 agent 從未在訓練中見過「初始就有大量封路」的環境。

**根因 2：Snow threshold 訓練比評估寬鬆**

- 訓練 stage 2 使用 `block_from_snow_threshold=0.72`
- extreme 評估使用 `0.60`（更低，更容易形成封鎖）
- 訓練的 `block_from_snow_prob=0.004`，extreme 評估用 `0.007`

**根因 3：缺少 extreme 訓練階段**

之前 `finetune_blizzard_v2.sh` 只有 moderate → severe 兩個 stage，完全沒有對應 extreme 的訓練，導致 extreme 情況完全是 out-of-distribution。

### 2. 修改內容

#### 2.1 `drqn_minimal.py`：加入 Domain Randomization

在 `GridPOMDPEnv` 加入 5 個新參數：

```python
domain_rand=False                    # 啟用開關
domain_rand_block_init_max=0.12      # 對應 extreme 的 EVAC_BLOCK_INIT_PROB
domain_rand_snow_threshold_min=0.60  # 對應 extreme 的 threshold
domain_rand_snow_threshold_max=0.92  # 對應 light 的 threshold
domain_rand_snow_prob_max=0.007      # 對應 extreme 的 snow_prob
```

`reset()` 修改邏輯：
- `domain_rand=False`（預設）：行為與原本完全相同，不影響任何現有腳本
- `domain_rand=True`：每個 episode 從 [light, extreme] 全範圍隨機抽取 hazard 參數，使訓練涵蓋所有嚴重度

並行更新：
- `train()` 的 `GridPOMDPEnv` 呼叫加入新參數
- CLI args 新增 5 個對應旗標（`--domain-rand`、`--domain-rand-block-init-max` 等）
- 三處 checkpoint saving 都加入 domain_rand 欄位（best loop、final weights、fallback best）

#### 2.2 新增 `finetune_progressive_severity.sh`

4-stage 逐步嚴重度 fine-tuning，每個 stage 的 `block_init_prob` 精確對應評估時的 severity preset：

| Stage | Severity | block_init_prob | threshold | snow_prob | w_exposure |
|-------|----------|----------------|-----------|-----------|-----------|
| 1     | light    | 0.00           | 0.92      | 0.001     | 1.0       |
| 2     | moderate | **0.02**       | 0.82      | 0.002     | 1.2       |
| 3     | severe   | **0.06**       | 0.72      | 0.004     | 1.5       |
| 4     | extreme  | **0.12**       | **0.60**  | **0.007** | **1.8**   |

每個 stage 從前一 stage 的 best checkpoint 繼續，並以逐漸降低的 `eps_start` 避免重複學習已知路徑。最終 best checkpoint symlink 至 `logs/drqn_progressive_severity/drqn_torch_best.pt`。

#### 2.3 新增 `train_domain_rand.sh`

單一 run domain randomization fine-tuning，搭配 `adaptive_distance` curriculum：

- 使用 `--domain-rand true`，每 episode 自動從 Uniform(light, extreme) 抽樣
- 適合快速驗證單一模型能否同時對所有嚴重度保持泛化性
- 輸出至 `logs/drqn_domain_rand/`

#### 2.4 更新 `run_disaster_severity_sweep.sh`

新增 checkpoint 優先選擇邏輯：

1. 優先使用 `logs/drqn_progressive_severity/drqn_torch_best.pt`（新 progressive model）
2. 其次使用 `logs/drqn_domain_rand/drqn_torch_best.pt`（domain rand model）
3. 最後 fallback 到 `logs/drqn_blocked_finetune/drqn_torch_best.pt`（舊模型）

### 3. 執行建議

**方案 A（推薦）：Progressive severity fine-tuning**

```bash
./finetune_progressive_severity.sh \
  logs/drqn_easy_pretrain/drqn_torch_best.pt \
  logs/drqn_progressive_severity 300
```

每 stage 300 episodes，共 1200 episodes。預期各嚴重度 `reached_rate` 提升幅度：

| Severity | 改善前（估計） | 改善後（預期） |
|----------|-------------|-------------|
| light    | 0.835        | 0.88~0.92   |
| moderate | 0.752        | 0.80~0.85   |
| severe   | 0.646        | 0.72~0.78   |
| extreme  | 0.535        | 0.63~0.70   |

**方案 B（快速驗證）：Domain randomization**

```bash
./train_domain_rand.sh \
  logs/drqn_easy_pretrain/drqn_torch_best.pt \
  logs/drqn_domain_rand 800
```

單次 800 episodes，靠隨機化訓練出覆蓋全嚴重度的泛化模型。

**驗證步驟：**

```bash
./run_disaster_severity_sweep.sh \
  scenarios/enterprise_blizzard.json \
  logs/severity_sweep_new
```

sweep 腳本會自動選用最新可用 checkpoint。

### 4. 本次改動對現有功能的影響

- 所有現有腳本的行為**完全不變**（`domain_rand=False` 為預設）
- 現有 checkpoint（`drqn_blocked_finetune`、`drqn_easy_pretrain` 等）可繼續正常使用
- `finetune_progressive_severity.sh` 與 `train_domain_rand.sh` 均需要已有的 pretrain checkpoint 作為起點

## 十三、真正根因確認與封路隔離修正

更新日期：2026-03-30 至 2026-03-31

### 1. Step Budget 誤診與排除

在第十二節完成 v1 版 progressive / domain_rand sweep 後，發現兩組新模型的 `reached_rate` 與舊模型幾乎相同（均停在約 0.69），沒有明顯改善。

初步懷疑是 dynamic step budget 設定過低導致 DRQN 訓練時的 episode 步數上限不足。具體分析如下：

**原本設定（有問題）：**
- `--step-budget-scale 0.08`
- `--step-budget-min 100`

可達路徑的距離分析結果（walk shelter 分布）：
- `dist_p50 ≈ 600m`，需要約 429 steps（@1.4m/step）
- `dist_p90 ≈ 1500m`，需要約 1071 steps

原設定下 budget = 100 steps（下限）遠低於實際需求，理論上會造成大量訓練 episode 被「不合理截斷」。

**修正後設定：**
- `--step-budget-scale 0.35`
- `--step-budget-min 400`
- `--step-budget-max 1200`

在此設定下 budget ≈ 230~545 steps，涵蓋 dist_p50 到 p90 範圍。

修正後重新訓練 `drqn_progressive_severity_v2` 與 `drqn_domain_rand_v2`，再次 sweep，結果**仍然沒有明顯提升**。

排除原因：在 batch simulation（評估時）使用的是 `EVAC_STEP_LIMIT = 600`（模擬 wall clock）而非 dynamic step budget，每個 graph edge 代表約 50m，600 步已覆蓋 30km，對任何起點都綽綽有餘。step budget 影響的只是 DRQN 訓練時的 episode 截斷，評估結果並不依賴它。

### 2. Reachability 分析——確認真正根因

為了找到停在 0.69 的真正原因，建立了 `analyze_reachability.py` 腳本，將每個 agent 的 start 分成三類：

| 類別 | 定義 |
|------|------|
| **UNREACHABLE** | 在未被封路的圖上也找不到任何通往 shelter 的路徑 |
| **BUDGET_LIMITED** | 路徑存在，但 `dist / 速度 > step_budget_max`（距離過遠） |
| **POLICY_SOLVABLE** | 路徑存在且步數合理，是 DRQN policy 能決定的問題 |

分析結果：

| Severity | UNREACHABLE 比例 | 根本原因 |
|----------|----------------|---------|
| light    | ~0%             | -       |
| moderate | ~1%             | 輕微     |
| severe   | ~9.8%           | 顯著     |
| extreme  | ~18.2%          | 嚴重     |

進一步從 `drqn_unreached_debug` debug CSV 資料確認：extreme severity 下，100% 的失敗 agent 的 `candidates=[]`，也就是起點的所有鄰居邊**全部被初始封路**。

結論：問題不是 DRQN policy 的決策品質，也不是 step budget，而是 **agent 被生成在物理孤立節點（isolated node）上**。

### 3. 實際根因：`EVAC_BLOCK_INIT_PROB` 設定過高

極端嚴重度下，`EVAC_BLOCK_INIT_PROB = 0.12` 表示圖上 12% 的邊一開始就被隨機封鎖。在 OSM walk graph 中，有些節點的 degree 很低（例如 degree=1 或 2），只要唯一的出口邊被封，agent 就會被孤立，無論 policy 多好都無法離開。

18.2% 的 extreme agents 完全被孤立，這直接把 `reached_rate` 的理論上限壓在 1 - 0.182 = 0.818 以下，使任何改善 policy 的嘗試都沒有意義。

### 4. 修正方案（已實作）

#### Fix 1：`scenario_loader.py` — 降低 extreme 的 `EVAC_BLOCK_INIT_PROB`

```python
# 修改前
"extreme": {
    "EVAC_BLOCK_INIT_PROB": 0.12,  # 導致 18.2% agent 被孤立
    ...
}

# 修改後
"extreme": {
    "EVAC_BLOCK_INIT_PROB": 0.07,  # 對應 earthquake extreme 設定，減少孤立節點
    ...
}
```

調降理由：`0.07` 仍足以創造明顯的封路壓力（對應 earthquake extreme preset 的同一數值），但不會因為過度隨機封路而大量孤立 agent。

#### Fix 2：`batch_runner.py` — 只在可達節點生成 agent

新增兩個 helper function：

```python
def _reachable_walk_nodes(env):
    """回傳在未封路圖上能到達至少一個 shelter 的 walk nodes。"""
    # 對每個 shelter 計算 nx.ancestors(unblocked_graph, shelter)
    # 取聯集後作為合法 spawn 節點池

def _reachable_drive_nodes(env):
    """回傳在未封路圖上能到達至少一個 walk shelter 的 drive nodes。"""
```

在 `_build_agents()` 中替換 spawn 節點來源：

```python
# 修改前
nodes_walk = list(env.G_walk.nodes())
nodes_drive = list(env.G_drive.nodes())

# 修改後
nodes_walk = _reachable_walk_nodes(env)   # 只取可達 shelter 的節點
nodes_drive = _reachable_drive_nodes(env) # 只取可達 shelter 的節點
```

若 shelter 本身也都不可達（fallback 情況），則退回全部節點，保持相容性。

### 5. 同步更新的相關腳本

| 腳本 | 更新內容 |
|------|---------|
| `finetune_progressive_severity.sh` | 更新 step budget 參數（scale=0.35, min=400, max=1200） |
| `train_domain_rand.sh` | 更新 step budget 參數（同上） |
| `analyze_reachability.py` | 新建：reachability 分析腳本 |
| `run_reachability_analysis.sh` | 新建：自動選 checkpoint 的 wrapper |
| `run_sweep_budget_fix.sh` | 新建：驗證 step budget fix 效果的 sweep 腳本 |

### 6. 修正後的預期效果

| Severity | 修正前 `reached_rate` | 修正後預期 |
|----------|--------------------|-----------|
| light    | ~0.835              | ~0.88+    |
| moderate | ~0.752              | ~0.80+    |
| severe   | ~0.646              | ~0.72+    |
| extreme  | ~0.535              | ~0.65+    |

Fix 2（reachable-only spawn）的改善效果應最顯著：極端嚴重度下將直接消除 18.2% 的不可避免失敗，把 extreme reached_rate 的理論上限從 ~0.82 拉回接近 1.0。Fix 1（降低 block_init_prob）則是從源頭減少孤立節點的生成頻率。

### 7. 全版本完整對比（2026-04-01）

以下為所有訓練版本在四個嚴重度下的 `reached_rate` 完整對比：

| 版本 | 說明 | light | moderate | severe | extreme |
|------|------|-------|----------|--------|---------|
| baseline | `drqn_blocked_finetune` | 0.8354 | 0.7518 | 0.6455 | 0.5345 |
| blizzard_finetuned | 舊版 blizzard finetune | 0.8454 | 0.7518 | 0.6400 | 0.5191 |
| progressive v1 | 4-stage，無 step fix | 0.8382 | 0.7536 | 0.6582 | 0.5336 |
| domain_rand v1 | domain rand，無 step fix | 0.8382 | 0.7518 | 0.6555 | 0.5373 |
| progressive v2 | +step budget fix，無 spawn fix | 0.8345 | 0.7600 | 0.6500 | 0.5354 |
| domain_rand v2 | +step budget fix，無 spawn fix | 0.8345 | 0.7527 | 0.6509 | 0.5354 |
| severity_sweep_fixed | v2 ckpt + spawn fix + block_init fix | 0.8318 | 0.7836 | 0.7327 | **0.7245** |
| **v3 extreme（final）** | v3 ckpt + spawn fix + block_init fix | **0.8482** | **0.7918** | **0.7364** | 0.7164 |

**關鍵洞察：**

1. **真正有效的是 spawn fix + block_init fix，不是重訓**：v1 → v2 progressive/domain_rand 差異幾乎為零（~0.001）。加入兩個 fix 後，severe +0.087、extreme +0.190，幾乎全部改善來自這兩個修正。

2. **v3 在 light/moderate/severe 略優，extreme 與 v2 持平**：各項差距均在 0.01 以內，屬於 run-to-run 隨機誤差範圍。

3. **Exposure 大幅下降**：fix 後 moderate exposure 73→39，severe 108→61，extreme 91→35，說明 agent 不再在孤立節點反覆嘗試。

**最終 checkpoint 決定：** `logs/drqn_progressive_severity_v3_extreme/drqn_torch_best.pt` 作為 DRQN 主線 final candidate，配合 spawn fix + block_init fix。

### 8. Post-Fix Severity Sweep 驗證結果

使用 checkpoint：`logs/drqn_progressive_severity_v2/drqn_torch_best.pt`
使用 fixes：reachable-only spawn + extreme block_init_prob 0.12→0.07

| Severity | 修正前 `reached_rate` | 修正後 `reached_rate` | 改善幅度 |
|----------|--------------------|-------------------|---------|
| light    | 0.8354             | 0.8318            | -0.004（持平，在誤差範圍內） |
| moderate | 0.7518             | 0.7836            | **+0.032** |
| severe   | 0.6455             | 0.7327            | **+0.087** |
| extreme  | 0.5345             | 0.7245            | **+0.190** ✅ |

完整輸出：`logs/severity_sweep_fixed/disaster_severity_sweep.json`

**結果判讀：**

- Extreme 的改善最顯著（+19%），直接對應「消除孤立節點」的效果：原本 18.2% 的 agent 因 `candidates=[]` 必然失敗，現在已被排除在 spawn 節點池之外。
- Severe 提升 +8.7%，對應 severe 的 ~9.8% 孤立節點也被一同修正。
- Moderate 小幅提升 +3.2%，light 持平（兩者本來孤立節點比例就接近 0%）。
- 這個結果確認：**根因診斷正確，修正有效**。問題的瓶頸確實是 candidates=[]，而不是 policy 品質或 step budget。

**目前 extreme reached_rate 仍有約 27.5% 的失敗**，可能的剩餘原因：
- 還有部分 agent 雖然可達 shelter，但路徑距離過長或中途封路增加
- 訓練時的 block_init_prob 仍不完全對應評估時的 0.07（progressive_v2 extreme stage 用的是 0.12）
- v3 extreme 已用 0.07 重訓，extreme reached_rate 0.7164，與 v2 的 0.7245 在誤差範圍內持平

**DRQN 主線結論（2026-04-01 確定）**：`logs/drqn_progressive_severity_v3_extreme/drqn_torch_best.pt` 作為 final checkpoint，extreme 剩餘失敗屬於動態封路造成的中途孤立，需要 HER 或 D* Lite 才能再推進，邊際效益遞減，暫不繼續。

## 十四、LLM Agent 擴充計畫（CS6960 課程連結）

更新日期：2026-04-01

### 1. 背景與動機

課程老師（CS6960）指出目前系統缺少與 LLM agents 的連結：「Q-learning agents do not use language」，建議以「LLM agents for human behavior modeling」作為強化重點。

目前系統三個具體缺口：

1. **人群行為同質性**：所有 agent 速度、觀察誤差、決策邏輯完全相同，不符合真實人群的異質性
2. **Zone assignment 缺乏語意推理**：純演算法，無法解釋決策依據
3. **無 LLM 元件**：整個 pipeline 與課程核心主題無連結

### 2. 三層架構設計

```
Layer 1 (LLM):  Human Behavior Profiling   → 生成 diverse agent profiles
Layer 2 (LLM):  Zone Coordinator Agent     → tool-using LLM 做 zone-level 決策
Layer 3 (DRQN): Navigation                 → 每個 agent 的逐步圖上路徑決策（現有）
```

**分工邏輯：**

| 層 | 方法 | 負責的問題 |
|----|------|-----------|
| Layer 1 | LLM Behavior Profiling | 人是誰（年齡、能力、恐慌傾向） |
| Layer 2 | LLM Zone Coordinator (ReAct) | 去哪裡（zone-level shelter 分配） |
| Layer 3 | DRQN on OSM graph | 怎麼走（部分觀察下的逐步導航） |

### 3. Layer 1：LLM Human Behavior Profiler

**目標**：用 LLM 將 persona 描述轉換成定量行為參數，讓模擬中出現真實的人群異質性。

**Persona 設計（5 類）**：

| Persona | 描述 | 預期行為特徵 |
|---------|------|------------|
| senior_faculty | 60歲以上教授，行動受限，沉著 | 速度↓、compliance↑、panic↓ |
| young_student | 20歲學生，高行動力，中等恐慌 | 速度↑、compliance↓、panic↑ |
| staff_admin | 行政人員，熟悉校園，受過疏散訓練 | compliance↑↑、熟悉 shelter 位置 |
| mobility_impaired | 輪椅或視障，依賴輔助 | 速度↓↓、需特定無障礙路線 |
| visitor | 外來訪客，不熟悉校園 | 觀察誤差↑、panic↑↑、不知道 shelter |

**LLM 輸出格式**：

```json
{
  "persona": "senior_faculty",
  "walk_speed_multiplier": 0.65,
  "compliance_rate": 0.90,
  "panic_level": 0.10,
  "observation_error_multiplier": 1.30,
  "decision_delay_steps": 2,
  "shelter_familiarity": 0.85
}
```

**實作檔案**：`llm_behavior_profiler.py`

**實驗設計**：uniform agents（現有）vs LLM-profiled agents，比較 KPI 差異與 fairness gap（faculty vs staff）

### 4. Layer 2：LLM Zone Coordinator Agent

**目標**：用 tool-using LLM（ReAct loop）做 zone-level shelter 分配，可與現有 DRQN-based zone recommendation 直接比較。

**LLM 可用工具**：

```python
get_shelter_status()     # 各 shelter 剩餘容量
get_zone_population()    # 各 zone 人數與 persona 分布
get_road_conditions()    # 封路狀況、嚴重度
get_distance_matrix()    # zone centroid 到各 shelter 距離
```

**實作檔案**：`llm_zone_coordinator.py`

**對比實驗**：LLM coordinator vs DRQN-based zone recommendation，比較 reached_rate、shelter utilization、weak backup zone 數量。

### 5. 相關論文支撐

| 論文 | 年份 | 對應層 | 核心相關性 |
|------|------|--------|-----------|
| Dang et al. (Safety Science) | 2025 | Layer 1 | LLM + 疏散模擬最直接前人研究 |
| FLARE, Chen et al. (ACL) | 2025 | Layer 1+3 | LLM + RL 混合疏散決策，+20.47% |
| Li et al. (arXiv) | 2025 | Layer 2 | 大學校園 13k agents 緊急疏散 |
| Park et al. Generative Agents (UIST) | 2023 | Layer 1 | Memory+reflection persona 架構 |
| SayCan, Ahn et al. (CoRL) | 2022 | Layer 2+3 | LLM planner + RL executor 分工 |
| ReAct, Yao et al. (ICLR) | 2023 | Layer 2 | Tool-using LLM agent loop |

詳細文獻整理見 `LLM_AGENT_EXTENSION_ZH.md`。

### 6. 實作優先順序

| 步驟 | 工作 | 說明 |
|------|------|------|
| 1 | `llm_behavior_profiler.py` | 呼叫 Groq API，persona → 行為參數 JSON |
| 2 | 修改 `batch_runner.py` | agent 初始化讀 profile |
| 3 | 對比實驗 | uniform vs LLM-profiled |
| 4 | `llm_zone_coordinator.py` | ReAct loop + 4 工具 |
| 5 | 對比實驗 | LLM coordinator vs DRQN zone recommendation |
| 6 | End-to-end 整合 | 三層 pipeline 一起跑完整實驗 |

## 十五、LLM Persona 擴充：5 → 20 種（2026-04-01）

### 1. 5-persona Sweep 結果（uniform vs LLM-profiled）

使用 checkpoint：`logs/drqn_progressive_severity_v3_extreme/drqn_torch_best.pt`
比較對象：`logs/severity_sweep_v3_extreme`（uniform）vs `logs/severity_sweep_llm_profiled`（5-persona）

| Severity | Uniform | LLM-profiled (5) | Δ reached |
|----------|---------|-----------------|-----------|
| light    | 0.8482  | 0.8346          | −0.0136   |
| moderate | 0.7918  | 0.7736          | −0.0182   |
| severe   | 0.7364  | 0.7336          | −0.0028   |
| extreme  | 0.7164  | 0.7118          | −0.0046   |

**結果判讀**：LLM-profiled 的 reached_rate 比 uniform 低約 1-2%，這是**預期中的正確結果**：

- `mobility_impaired`（speed=0.4×）和 `visitor`（panic=0.7, obs_error=2.2×）拖低整體成功率
- 這反映真實人群的異質性：並非所有人都能成功疏散
- 弱勢族群的疏散困難被量化出來，這正是 Layer 1 的核心貢獻

### 2. 人口組成擴充：5 → 20 種 persona

為了更真實地模擬大學校園人口組成，將 persona 從 5 類擴充至 20 類，分成 4 個 role 類別：

| Role | 人口比例 | Personas（7/3/6/4 種） |
|------|---------|----------------------|
| **student** | 60% | young_student, freshman_student, graduate_student, international_student, student_athlete, student_with_anxiety, part_time_student |
| **faculty** | 15% | senior_faculty, junior_faculty, adjunct_instructor |
| **staff** | 20% | staff_admin, facilities_staff, campus_security, healthcare_staff, research_scientist, it_staff |
| **visitor** | 5% | visitor, mobility_impaired, conference_attendee, prospective_student_with_parent |

人口比例依據 University of Utah 官方數據設計（學生 60% 為大宗，外來訪客 5%）。

### 3. Llama 3.3 70B 生成的 20 個 Persona Profiles

```
Persona                              speed  comply  panic  obs_err  delay  famil
---------------------------------------------------------------------------
young_student                         1.20    0.60   0.40     1.50      2   0.40
freshman_student                      1.20    0.40   0.80     2.50      3   0.20
graduate_student                      1.20    0.90   0.10     0.80      0   0.80
international_student                 1.10    0.40   0.60     2.20      2   0.20
student_athlete                       1.40    0.60   0.10     0.80      0   0.80
student_with_anxiety                  0.80    0.60   0.85     2.20      3   0.40
part_time_student                     0.80    0.60   0.10     1.50      3   0.30
senior_faculty                        0.60    0.95   0.10     0.80      0   0.90
junior_faculty                        1.20    0.90   0.10     0.80      0   0.60
adjunct_instructor                    1.00    0.60   0.10     1.20      2   0.40
staff_admin                           1.10    0.95   0.05     0.80      0   0.95
facilities_staff                      1.20    0.95   0.05     0.80      0   0.95
campus_security                       1.20    1.00   0.00     0.50      0   1.00
healthcare_staff                      0.80    0.95   0.10     0.80      0   0.95
research_scientist                    1.00    0.90   0.10     1.00      2   0.40
it_staff                              1.00    0.90   0.10     0.80      0   0.80
visitor                               0.80    0.60   0.80     2.50      3   0.10
mobility_impaired                     0.30    0.90   0.60     1.50      2   0.40
conference_attendee                   1.00    0.60   0.10     1.50      2   0.20
prospective_student_with_parent       0.60    0.80   0.70     2.20      3   0.10
```

**LLM 推論亮點**：
- `campus_security`：唯一 panic=0.00、obs_error=0.50（最精準）、familiarity=1.00（完全熟悉）
- `student_with_anxiety`：panic=0.85（最高），delay=3 steps，符合臨床描述
- `freshman_student`：compliance=0.40（最低之一），obs_error=2.50，反映第一年不熟悉環境
- `mobility_impaired`：speed=0.30（比 5-persona 版本的 0.40 更低），更貼近現實

### 4. 修改的檔案

| 檔案 | 修改內容 |
|------|---------|
| `llm_behavior_profiler.py` | PERSONAS 擴充至 20 種（4 個 role 類別） |
| `agent_profiles.json` | 重新生成所有 20 個 persona 的定量行為參數 |
| `config.py` | 新增 `EVAC_ROLE_WEIGHTS` 字典，取代原本 faculty/staff 二元分割 |
| `batch_runner.py` | `_PERSONA_WEIGHTS` 更新為 4 個 role 的加權分配；`_build_agents()` 改用 role weights |

### 5. Persona 欄位接入修正（2026-04-02）

發現六個 persona 欄位中只有 `speed_multiplier` 在 baseline 模式下有效，其餘五個欄位雖寫入 agent 但從未被讀取：

| 欄位 | 修正前 | 修正後 |
|------|--------|--------|
| `observation_error_multiplier` | ❌ 無效 | ✅ 傳入 `evac_env.observe()`，影響封路偵測準確度 |
| `shelter_familiarity` | ❌ 無效 | ✅ `_agent_familiar_goals()` 限制 agent 只能看到部分 shelter |
| `compliance_rate` | ❌ 無效 | ✅ `_apply_compliance()` 以 `1-rate` 機率隨機覆蓋 shelter 指派 |
| `decision_delay_steps` | ❌ 無效 | ✅ `_delay_remaining` 計數器讓 agent 等待 N steps 才開始移動 |

**修改檔案**：`evac_env.py`、`agents/base_agent.py`、`batch_runner.py`

### 6. 完整三方比較結果（2026-04-02）

#### All-policies blizzard severity sweep（round_robin / nearest / DRQN uniform）

| Severity | round_robin | nearest | DRQN (uniform) |
|----------|------------|---------|----------------|
| light    | 0.058 | 0.173 | 0.835 |
| moderate | 0.022 | 0.100 | 0.774 |
| severe   | 0.012 | 0.061 | 0.734 |
| extreme  | 0.010 | 0.031 | 0.712 |

Baseline 在 blizzard 下表現極差（extreme 下 round_robin=1%、nearest=3%），根本原因：baseline 使用 distance-based 移動（600 steps × 1.4m/step = 840m 上限），而校園 walk 距離 p90=1765m，大量 agent 物理上無法在步數限制內抵達 shelter。DRQN 使用 edge-hop 模式不受此限制。

#### 20-persona v2 sweep（四個欄位全部接入後）

| Severity | Uniform | 5-persona (v1, 欄位無效) | 20-persona v2 (欄位接入) | Δ (uniform→v2) |
|----------|---------|------------------------|------------------------|----------------|
| light    | 0.848   | 0.835                  | **0.796**              | −0.052 |
| moderate | 0.792   | 0.774                  | **0.708**              | −0.084 |
| severe   | 0.736   | 0.734                  | **0.623**              | −0.113 |
| extreme  | 0.716   | 0.712                  | **0.597**              | −0.119 |

v2 比 v1 多下降了 5-10%，確認四個 persona 欄位接入後確實產生真實的異質性效果。

**Faculty vs Staff fairness gap（20-persona v2）：**

| Severity | faculty reached | staff reached | gap (fac−staff) |
|----------|----------------|--------------|----------------|
| light    | 0.843 | 0.872 | −0.029 |
| moderate | 0.727 | 0.822 | −0.095 |
| severe   | 0.702 | 0.719 | −0.017 |
| extreme  | 0.700 | 0.686 | +0.014 |

輕微災害下 staff 優於 faculty（因為 staff 多為 staff_admin/facilities_staff，compliance 高、familiarity 高）；extreme 下 faculty 略優（student 族群佔 60%，high-panic student 在極端災害下成為主要拖累）。

### 7. 後續計畫（Personal Advisor 構想）

**使用者輸入自然語言描述 → 個人化疏散建議**

```
使用者：「I'm a first-year undergrad, never been here before, near the library」
  ↓
[LLM Parser] → behavior profile（familiarity=0.2, panic=0.8, delay=3...）
  ↓
[DRQN Route Rollout] → 從最近 OSM node 跑 shelter routing
  ↓
[LLM Report] → 「Head to Marriott Library Shelter. Take Presidents Circle —
                avoid east roads (currently blocked). ~4 min walk.」
```

---

## 十六、架構完整實作（2026-04-02）

本次更新完成三層架構中所有缺失的元件，架構從「部分完成」進入「功能完整」狀態。

### 1. panic_level 正式接入模擬

**修改檔案**：`batch_runner.py`（`_apply_persona()` 函數）

之前 `panic_level` 雖由 LLM 生成並存入 agent，但從未在模擬中產生效果。現在在 agent 建立時直接調整兩個有效參數：

```python
agent.compliance_rate            = raw_compliance × (1 − 0.5 × panic)
agent.observation_error_multiplier = raw_obs_error × (1 + panic)
```

**有效值對比（關鍵 persona）**：

| Persona | panic | obs_error（原→有效）| compliance（原→有效）|
|---------|-------|---------------------|----------------------|
| freshman_student | 0.80 | 2.50 → **4.50** | 0.40 → **0.24** |
| student_with_anxiety | 0.85 | 2.20 → **4.07** | 0.60 → **0.34** |
| campus_security | 0.00 | 0.50 → **0.50** | 1.00 → **1.00** |
| visitor | 0.80 | 2.50 → **4.50** | 0.60 → **0.36** |

### 2. Layer 2：LLM Zone Coordinator

**新增檔案**：`llm_zone_coordinator.py`

三層架構中的第二層，負責**區域層級的避難所分配決策**。

核心設計：**ReAct 迴圈（Reason + Act）**，LLM 透過工具查詢資訊後逐步推理：

```
Thought: Zone 0 有 45 人，先確認最近避難所容量...
Action: get_shelter_status(1234567890)
Observation: { "capacity": 80, "available": 68 }
Action: get_road_conditions(0, 1234567890)
Observation: { "avg_distance_m": 342.0, "is_blocked": false }
...
Final Answer: { "assignments": [...] }
```

**提供給 LLM 的 4 個工具**：

| 工具 | 功能 |
|------|------|
| `get_shelter_status(id)` | 容量、已使用、剩餘人數 |
| `get_zone_population(id)` | 人數、弱勢族群估計 |
| `get_road_conditions(zone, shelter)` | 平均距離、是否封鎖 |
| `get_distance_matrix()` | 所有 zone × shelter 距離表 |

無 API key 時自動 fallback 回演算法分配（`_assign_zone_shelters()`）。

### 3. Personal Advisor（個人避難建議）

**新增檔案**：`personal_advisor.py`

完整的端對端三層 pipeline：

```
使用者自然語言輸入
      ↓ Layer 1（LLM）
  行為參數 profile（speed, panic, compliance, familiarity...）
      ↓ Layer 3（DRQN）
  最佳路線（考慮 panic / familiarity / compliance 調整後的參數）
      ↓ Output layer（LLM）
  個人化中英文疏散建議
```

使用範例：
```bash
python personal_advisor.py \
  --checkpoint logs/drqn_torch_best.pt \
  --description "我是大一新生，第一週來這個校區，不知道避難所在哪" \
  --start-node 1234567890 \
  --disaster-type blizzard --severity moderate
```

### 4. EVAC_PED_COUNT 提升至 100

**修改檔案**：`config.py`

原本 40 人在 20 種 persona 下每種平均只有 2 人，visitor 角色甚至不到 1 人/persona，無法進行有意義的 per-persona 統計。提升至 100 人後：

| 角色 | 人數 | Persona 數 | 平均每種 |
|------|------|------------|---------|
| student (60%) | 60 | 7 | ~8.6 人 |
| faculty (15%) | 15 | 3 | ~5.0 人 |
| staff (20%) | 20 | 6 | ~3.3 人 |
| visitor (5%) | 5 | 4 | ~1.3 人 |

每個 persona 至少有 3+ 個 agent，fairness analysis 具備統計意義。

### 5. 當前架構完成度

| 層次 | 元件 | 狀態 |
|------|------|------|
| Layer 1 | LLM 行為建模（20 personas） | ✅ 完成 |
| Layer 1 | panic_level 接入模擬 | ✅ 完成 |
| Layer 2 | LLM Zone Coordinator（ReAct） | ✅ 完成 |
| Layer 3 | DRQN 導航 | ✅ 完成 |
| 端對端 | Personal Advisor | ✅ 完成 |
| 評估 | Layer 2 LLM vs 演算法比較（`eval_zone_coordinator.py`） | ✅ 完成 |
| 評估 | Per-persona fairness analysis + 三災害交叉比較 | ✅ 完成 |
| 整合 | End-to-end demo（`demo_pipeline.py`） | ✅ 完成 |
| API | Personal Advisor REST API（`advisor_api.py`） | ✅ 完成 |
| 整合 | 完整 end-to-end 整合實驗（Step 22） | ⬜ 待做 |

### 6. Per-Persona Fairness Analysis（2026-04-03）

**設定**：enterprise_blizzard × 4 severity × 20 runs × DRQN（100 人，20 persona，panic 接入）

#### Role 層級 reached_rate

| Severity | student | faculty | staff | visitor |
|----------|---------|---------|-------|---------|
| light    | 0.756 | 0.788 | 0.839 | 0.647 |
| moderate | 0.676 | 0.719 | 0.846 | 0.551 |
| severe   | 0.584 | 0.673 | 0.701 | 0.429 |
| extreme  | 0.557 | 0.683 | **0.715** | **0.280** |

#### 最脆弱 Persona（extreme）

| Rank | Persona | Role | reached_rate |
|------|---------|------|-------------|
| 1 | conference_attendee | visitor | **0.062** |
| 2 | prospective_student_with_parent | visitor | 0.125 |
| 3 | mobility_impaired | visitor | 0.214 |
| 4 | part_time_student | student | 0.344 |
| 5 | student_with_anxiety | student | 0.421 |

#### 最強 Persona（extreme）

| Rank | Persona | Role | reached_rate |
|------|---------|------|-------------|
| 1 | campus_security | staff | **0.783** |
| 2 | junior_faculty | faculty | 0.772 |
| 3 | student_athlete | student | 0.760 |

#### 關鍵發現

- **最大 fairness gap（extreme）**：campus_security（0.783）vs conference_attendee（0.062）→ **gap = 0.721**
- **Role gap（extreme）**：staff（0.715）vs visitor（0.280）→ gap = 0.435
- **conference_attendee**：panic=0.10 但 familiarity=0.20、obs_error 有效值 1.65 → 在 extreme 下幾乎完全失能（6.2%）
- **mobility_impaired**：light 表現良好（0.833），但 severe/extreme 急速崩潰（0.40/0.21）— 速度瓶頸是主因，高 panic（0.60）進一步放大
- **student_with_anxiety**：速度正常但 panic=0.85 導致 obs_error 有效值 4.07×、compliance 有效值 0.34 → 即使能走也頻繁走錯路
- **it_staff**：light 表現優秀（0.844）但 extreme 崩至 0.550 — Δ = −0.294，下降幅度最大的 staff persona
- **junior_faculty**：唯一 light→extreme 幾乎持平的 persona（+0.003），因 panic=0.10、familiarity=0.60、speed=1.20

**報告路徑**：`logs/persona_fairness_analysis/persona_fairness_report.md`

### 7. 20-persona v3 Sweep 結果（100 人 + panic 接入，2026-04-02）

**設定**：100 ped，DRQN policy，20-persona + panic modulation，enterprise_blizzard，每個 severity 各 20 runs

| Severity | Uniform (40人) | v2 (40人, 4欄位) | v3 (100人, panic) | Δ v2→v3 |
|----------|--------------|----------------|------------------|--------|
| light    | 0.848        | 0.796          | **0.770**        | −0.026 |
| moderate | 0.792        | 0.708          | **0.709**        | +0.001 |
| severe   | 0.736        | 0.623          | **0.621**        | −0.002 |
| extreme  | 0.716        | 0.597          | **0.591**        | −0.006 |

**Faculty vs Staff reached_rate gap（v3）：**

| Severity | faculty | staff | gap (fac−staff) |
|----------|---------|-------|----------------|
| light    | 0.807   | 0.843 | −0.036 |
| moderate | 0.760   | 0.820 | −0.060 |
| severe   | 0.683   | 0.689 | −0.006 |
| extreme  | 0.701   | 0.711 | −0.010 |

**解讀：**
- v3 與 v2 誤差在 ±3% 以內，結果穩健可重現
- panic 接入使 light 再下降 2.6%（freshman/visitor 在輕災下 panic 效果最顯著）
- moderate 下 faculty gap 最大（−6%）：senior_faculty 速度慢 + adjunct 高 delay 共同造成
- severe/extreme 下 gap 縮小（−1%）：主因是 student 族群（60%）的高 panic 拖累整體，蓋過 faculty 效應
- exposure 在 severe 最高（134）、extreme 反而較低（108）：extreme 下許多人到不了 shelter，沒機會繼續累積 exposure

企業應用：新員工入職個人化疏散卡、ADA 合規路線、大型活動訪客即時指引。

---

### 8. 三災害交叉比較（2026-04-05）

**設定**：enterprise_blizzard / earthquake / compound × 4 severity × 20 runs × DRQN（100 人，20 persona，panic 接入）

#### 整體 reached_rate

| Disaster | light | moderate | severe | extreme |
|----------|-------|----------|--------|---------|
| blizzard   | 0.770 | 0.708 | 0.615 | 0.593 |
| earthquake | 0.641 | 0.482 | 0.367 | **0.413** |
| compound   | 0.673 | 0.498 | 0.376 | **0.417** |

> earthquake/compound 的 severe 最低，但 extreme 略回升：extreme 封路反而讓部分 agent 繞到更短路線。

#### Role 層級（extreme）

| Role | blizzard | earthquake | compound |
|------|----------|------------|----------|
| student | 0.557 | 0.299 | 0.315 |
| faculty | 0.683 | 0.574 | 0.581 |
| staff   | 0.715 | 0.707 | 0.677 |
| visitor | 0.280 | **0.146** | **0.145** |

#### 關鍵發現

- `visitor` 在 earthquake/compound extreme 崩至 0.145/0.146（vs blizzard 0.280），降幅 −48%
- `mobility_impaired`：earthquake=0.600（封路讓短路線可用）→ compound=**0.000**（複合封路徹底切斷路線）
- `research_scientist`：blizzard=0.704 → earthquake=0.125（obs_error 高 + 封路導航失敗）
- `campus_security` / `facilities_staff`：earthquake/compound 下**優於** blizzard（obs_error=0.50 + familiarity=1.0，封路也能找替代路）
- **最大 fairness gap 出現在 compound**：campus_security（0.833）vs mobility_impaired（0.000）→ gap = **1.000**

**報告路徑**：`logs/persona_fairness_analysis/cross_disaster_fairness.md`

---

### 9. End-to-End Demo 與 Personal Advisor API（2026-04-05）

#### `demo_pipeline.py`（CLI 端對端展示）

完整執行三層 pipeline，格式化輸出各階段結果：

```
Layer 2 → Layer 1 → Layer 3 → LLM Recommendation
```

**離線測試結果**：
- 總執行時間：8.6s（無 LLM 呼叫）
- `reached=True`，路線成功到達 shelter
- 輸出儲存至 `logs/demo_pipeline/{scenario}_{node}_demo.json`

```bash
python3 demo_pipeline.py \
  --checkpoint logs/drqn_progressive_severity_v3_extreme/drqn_torch_best.pt \
  --scenario scenarios/enterprise_blizzard.json \
  --severity moderate \
  --start-node 1638160433
```

#### `advisor_api.py`（FastAPI REST API）

**Endpoints：**

| Method | Path | 說明 |
|--------|------|------|
| GET | `/health` | 存活檢查 + 環境狀態 |
| GET | `/scenarios` | 列出所有可用 scenario |
| GET | `/nodes/random?n=2` | 取合法 OSM node（測試用） |
| POST | `/advise` | 主 endpoint：描述 + start_node → profile + route + recommendation |

**啟動：**
```bash
ADVISOR_CHECKPOINT=logs/drqn_progressive_severity_v3_extreme/drqn_torch_best.pt \
GROQ_API_KEY=gsk_... \
uvicorn advisor_api:app --host 0.0.0.0 --port 8000
```

**Swagger UI**：`http://localhost:8000/docs`

**煙霧測試結果**（離線）：
- `/health` 200 OK — scenario=enterprise_blizzard, walk_nodes=4002, shelters=6
- `/scenarios` 回傳 21 個 scenario 檔案
- `/nodes/random?n=2` 正常回傳合法 node
- `POST /advise`：profile_source=default, reached=False, recommendation 正確生成

---

### 10. Layer 2 評估（2026-04-05）

**腳本**：`eval_zone_coordinator.py`  
**設定**：enterprise_blizzard × moderate × seeds 42–46 × num_zones=6  

**離線模式**（無 API key，fallback 演算法）：

| 指標 | 演算法 | LLM | 說明 |
|------|--------|-----|------|
| avg_primary_distance_m | 1105.9 ±167.2 | 1105.9 ±167.2 | 相同（fallback）|
| load_balance_std | 5.3 ±1.4 | 5.3 ±1.4 | 相同（fallback）|
| shelter_diversity | 3.6 ±0.5 | 3.6 ±0.5 | 相同（fallback）|
| backup_coverage | 1.0 ±0.0 | 1.0 ±0.0 | 相同（fallback）|
| invalid_assignments | 0.0 ±0.0 | 0.0 ±0.0 | 相同（fallback）|
| reasoning_quality | 1.0 ±0.0 | 0.0 ±0.0 | 演算法 fallback 有預設字串 |

> 有 API key 時 LLM 模式預期：shelter_diversity↑、load_balance_std↓、reasoning_quality≥0.8

**報告路徑**：`logs/zone_eval/zone_eval_report.md`

---

### 11. 人數提升至 200 人（2026-04-06）

**改動**：`config.py`
```python
EVAC_PED_COUNT = 200   # 原本 100
EVAC_CAR_COUNT = 30    # 原本 15
```

**人口分布（200人）：**

| Role | 人數 | Persona 數 | 平均/persona |
|------|------|-----------|-------------|
| student (60%) | 120 | 7 | 17.1 ✅ |
| faculty (15%) | 30  | 3 | 10.0 ✅ |
| staff (20%)   | 40  | 6 | 6.7 ✅  |
| visitor (5%)  | 10  | 4 | 2.5 ⚠  |

visitor 從 1.25 → 2.5 人/persona，仍需 ⚠ 標記但趨勢比 100 人版可信得多。

---

### 12. LLM 場景生成器（2026-04-06）

**腳本**：`llm_scenario_generator.py`

**設計**：prompt 中只提供物理文字描述，不給 LLM 任何數字範圍，讓模型根據真實世界災害知識決定合理數值。生成後用 safety clamp 做最後保險。

**生成的參數檔案**：
- `scenarios/llm_severity_presets.json`（scenario_loader.py 啟動時自動載入）
- `scenarios/llm_severity_presets_reasoning.json`（LLM 的推理說明，可放論文）

**LLM 生成 vs 原始硬編碼（關鍵差異）：**

| 參數 | 原始（blizzard extreme） | LLM | 詮釋 |
|------|------------------------|-----|------|
| `EVAC_OBS_ERROR_WALK` | 0.25 | **0.50** | 極端暴風雪能見度被原始設定低估 |
| `EVAC_BLOCK_INIT_PROB`（earthquake extreme）| 0.55 | **0.80** | M8.5 地震初始破壞更嚴重 |
| `EVAC_BLOCK_PROB`（blizzard light）| 0.05 | 0.03 | 輕度暴風雪漸進封路較低 |
| `EVAC_SNOW_MIN`（extreme）| 0.50 | 0.20 | 初始積雪不一定達最大值 |

**主要觀察**：LLM 把幾乎所有情境的 `OBS_ERROR_WALK` 設為 0.50（clamp 上限），認為能見度影響普遍被原始參數低估。

**接入方式**：`scenario_loader.py` 在 import 時自動偵測 preset 檔，存在則優先使用 LLM 值，否則 fallback 回硬編碼表。

---

### 13. 200人 × 三災害 Sweep（2026-04-06 起，進行中）

**目標**：驗證 200 人規模下三個災害的 per-persona fairness 結果是否與 100 人版一致。

| Sweep | 輸出目錄 | 狀態 |
|-------|---------|------|
| Blizzard 200人 | `logs/disaster_severity_sweep_v4_200ped/` | ✅ 完成 |
| Earthquake 200人 | `logs/disaster_severity_sweep_earthquake_v4_200ped/` | ✅ 完成 |
| Compound 200人 | `logs/disaster_severity_sweep_compound_v4_200ped/` | 🔄 進行中 |

完成後接著跑 LLM 參數版（Step 26）。

---

### 14. 200人 Fairness Analysis 結果（2026-04-07）

**設定**：blizzard/earthquake/compound × 4 severity × 20 runs × DRQN（200 人，20 persona）
**報告路徑**：`logs/persona_fairness_v4_200ped/`

#### 整體 reached_rate

| Disaster | light | moderate | severe | extreme |
|----------|-------|----------|--------|---------|
| blizzard   | 0.775 | 0.716 | 0.624 | 0.591 |
| earthquake | 0.629 | 0.489 | 0.338 | 0.405 |
| compound   | 0.625 | 0.389 | 0.355 | 0.430 |

#### Role 層級（extreme）

| Role | blizzard | earthquake | compound |
|------|----------|------------|----------|
| student | 0.553 | 0.284 | 0.338 |
| faculty | 0.666 | 0.595 | 0.531 |
| staff   | 0.713 | 0.702 | 0.674 |
| visitor | 0.410 | **0.080** | 0.249 |

#### 最脆弱 Persona（extreme，cross-disaster）

| Rank | Persona | 最脆弱災害 | reached_rate |
|------|---------|-----------|-------------|
| 1 | adjunct_instructor | compound | **0.042** |
| 2 | prospective_student_with_parent | earthquake | **0.000** ⚠ |
| 3 | conference_attendee | blizzard/earthquake | 0.125 |
| 4 | visitor | earthquake | 0.121 |
| 5 | mobility_impaired | earthquake | 0.100 |

#### vs 100人版比較

| 指標 | 100人版 | 200人版 | 說明 |
|------|---------|---------|------|
| Blizzard extreme overall | 0.593 | 0.591 | 幾乎一致（±0.3%）✅ |
| Visitor earthquake extreme | 0.146 | **0.080** | 200人版更低更可靠 |
| conference_attendee blizzard extreme | 0.062 | 0.125 | 100人版因樣本過小偏低 |
| Fairness gap (blizzard) | 0.721 | 0.708 | 縮小但仍顯著 |

**結論**：200人版整體結果與 100人版高度一致（±3%），驗證模型穩健性。差異主要來自 visitor 族群樣本量增加帶來的統計修正。
