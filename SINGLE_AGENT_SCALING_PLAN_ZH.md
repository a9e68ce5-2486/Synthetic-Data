# Single-Agent Scaling 計畫

> 文件建立日期：2026-03-22
>  
> 最近更新日期：2026-03-23
>  
> 本文件對應 blocked-stage 成功、multi-seed 驗證完成、baseline comparison 完成之後的下一階段規劃

這份文件整理目前專案在完成 easy-stage 與 blocked-stage DRQN 主線後，下一步如何進行 **single-agent scaling**。

這裡的 single-agent scaling 指的是：

- 仍然維持目前的 single-agent shelter-routing 訓練框架
- 不直接跳到 multi-agent RL
- 先把場景規模與壓力提高，觀察現有方法是否仍然有效

## 一、為什麼先做 single-agent scaling

目前主線已經完成：

- easy adaptive 訓練成功
- blocked finetune 成功
- evaluation 與動畫皆達到很高的 reached rate
- multi-seed evaluation 已確認 blocked final model 穩定
- DRQN 已正式優於 heuristic baselines

目前可作為 scaling 起點的模型為：

- `logs/drqn_blocked_finetune/drqn_torch_best.pt`

因此下一步最合理的問題不是：

- DRQN 還能不能在目前小規模場景成功

而是：

- 當人數、車輛數量、shelter 數量增加時，這條方法是否仍然有效

選擇先做 single-agent scaling，而不是直接跳到 multi-agent RL，有三個理由：

1. 風險較低
- 不會一下子把問題變成完全不同的訓練架構

2. 變因較少
- 比較容易知道效能下降是因為規模增加，還是因為模型真的不適合更大場景

3. 能為後續 multi-agent extension 提供明確證據
- 如果 scaling 後開始出現 congestion、resource competition、shelter overload 等問題
- 那就能合理說明下一步為什麼需要更多 agent-agent / agent-resource interaction modeling

## 二、single-agent scaling 的核心目標

single-agent scaling 的目的不是馬上重新設計整個 DRQN，而是先回答以下問題：

1. 現有最佳模型在更大規模下是否仍然能維持高 reached rate
2. 人數增加後，是否會出現更嚴重的 bottlenecks
3. shelter 數量增加後，是否能改善整體避難效率
4. 現有 policy 是否會因為更高密度環境而出現不公平現象

## 三、建議的 scaling 順序

不要一次把所有東西都放大，應該分階段來做。

### 第一階段：只增加 agent 數量

先固定：

- road network
- hazard setting
- shelter set

只增加：

- pedestrian 數量
- car 數量

例如：

- `ped: 40 -> 60 -> 80`
- `car: 15 -> 20 -> 30`

目的：

- 看現有 final DRQN 在更高需求下是否仍然穩定
- 觀察 congestion 與 bottleneck 是否明顯增加

### 第二階段：只增加 shelter 數量

在 agent 數量維持固定的情況下：

- 擴大 shelter candidate set
- 測試不同 shelter 數量配置

例如：

- 原 shelter set
- 原 shelter set + 2 個新 shelter
- 原 shelter set + 4 個新 shelter

目的：

- 看 shelter 數量是否能改善：
  - reached rate
  - t90 / t95
  - avg exposure
  - fairness

### 第三階段：同時增加 agents 與 shelters

在前兩階段都完成後，再測：

- 高 agent density
- 更多 shelters
- 相同 hazard setting

目的：

- 觀察資源增加是否能抵消高密度壓力
- 這時的結果最接近更真實的大規模避難情境

## 四、目前建議保留不變的部分

在 single-agent scaling 階段，以下內容先不要動：

1. DRQN 主結構
- 不重新設計 network
- 不先跳到 multi-agent RL

2. 主線訓練邏輯
- 保持目前已證明有效的：
  - adaptive curriculum
  - failure-aware exploration
  - checkpoint learning
  - blocked-aware replanning
  - frontier / revisit control

3. 核心 reward 設計
- 先不要再大改 reward
- 否則之後很難判斷效能變化來自 scaling 還是 reward 變動

## 五、single-agent scaling 應觀察的 KPI

在更大規模場景下，至少要持續追蹤以下指標：

### 1. Reached Rate

最基本的成功指標：

- 有多少 agent 最終到達 shelter

### 2. Alive Rate

如果後續 hazard 更強，alive rate 也會是必要指標。

### 3. Avg Exposure

即使 reached rate 高，也不能忽略暴露量：

- 是否因為繞路或壅塞導致 exposure 顯著上升

### 4. t90 / t95

看整體 evacuation 的速度是否因規模增加而明顯變慢。

### 5. Fairness Gap

尤其在更大規模下，要看：

- faculty / staff 的 shelter arrival rate 是否開始出現差距

### 6. Top Bottlenecks

放大場景後，最值得關注的就是：

- 哪些 edge 的通行壓力顯著上升
- 是否開始形成穩定瓶頸

## 六、最值得先回答的研究問題

single-agent scaling 階段最值得回答的不是「能不能做得更複雜」，而是：

1. 現有 DRQN 在更高 agent density 下是否仍然有效？
2. shelter 數量增加是否能顯著改善 KPI？
3. 規模擴大後，失效是來自：
   - routing policy 本身
   - congestion
   - shelter allocation
   - fairness

這些答案會直接影響下一步是否需要：

- 更明確的 congestion-aware features
- shelter-aware decision
- multimodal coordination
- 或真正的 multi-agent RL

## 七、與後續 multi-agent / multimodal extension 的關係

single-agent scaling 不是最終目標，而是下一階段更複雜系統的前置驗證。

如果 scaling 後發現：

- reached rate 開始下降
- bottlenecks 明顯惡化
- fairness gap 上升
- shelter loading 很不均

那就能合理支持下一步去做：

- shelter-aware policy
- vehicle-aware routing
- multimodal evacuation
- agent-resource interaction modeling

也就是說，single-agent scaling 的結果會告訴你：

- 目前 single-agent 方法可以撐到什麼程度
- 什麼時候真的有必要進一步升級成更完整的 multi-agent / multimodal decision system

## 八、建議的實作順序

### Step 1

建立第一個 larger-scale scenario：

- 增加 ped 數量
- 增加 car 數量
- shelter 不變

### Step 2

用目前最佳 blocked model 直接測：

- `logs/drqn_blocked_finetune/drqn_torch_best.pt`

先不要重訓。

### Step 3

觀察 KPI 是否下降，以及下降原因。

### Step 4

如果現有模型泛化不足，再考慮把 larger-scale scenario 納入訓練流程。

### Step 5

等單純 scaling 的結果清楚後，再決定是否進入：

- shelter-aware extension
- vehicle-aware extension
- multimodal extension
- multi-agent extension

## 九、目前建議

目前最合理的選擇是：

- 先做 single-agent scaling
- 不直接跳到 multi-agent RL

因為：

- 你現在已有一條成熟主線
- 先做 scaling 能最大化保留目前成果
- 也能為後續更複雜模型提供明確依據

因此下一階段的策略應該是：

1. 先擴大場景規模
2. 驗證現有 best model 的可擴展性
3. 再決定是否要進一步做 multimodal / multi-agent interaction

## 十、目前已完成的 scaling 驗證

更新日期：2026-03-23

目前已用：

- `logs/drqn_blocked_finetune/drqn_torch_best.pt`

完成一輪大範圍 single-agent scaling 測試，規模從：

- `60 ped / 20 car`

擴到：

- `500 ped / 200 car`

代表性結果如下：

- `60 / 20`
  - `reached = 0.9537`
  - `exposure = 4.0570`
  - `t95 = 146.4`

- `200 / 80`
  - `reached = 0.9902`
  - `exposure = 7.0167`
  - `t95 = 197.25`

- `500 / 200`
  - `reached = 0.9968`
  - `exposure = 5.9366`
  - `t95 = 159.0`

這表示：

- 目前 DRQN 在純數量擴張下仍維持很高的 reached rate
- 單靠 agent 數量放大，尚未完全逼出更真實的系統壓力

因此，scaling 階段的重點已經從：

- 純 agent count scaling

進一步轉向：

- interaction-aware scaling

也就是：

- shelter capacity
- congestion / queueing
- 資源競爭

## 十一、目前建議的下一步

目前最合理的 scaling 下一步不是再繼續無限制加大人數，而是：

1. 打開 shelter capacity
2. 重新跑 `200/80`、`300/120`、`500/200`
3. 觀察：
   - reached rate 是否開始下降
   - reassignments 是否上升
   - t95 / exposure 是否明顯惡化

也就是把 scaling 從「數量放大」推進到「互動壓力放大」。

## 十二、目前已完成的 capacity-aware scaling

更新日期：2026-03-23

目前已完成第一輪 shelter-capacity scaling，代表性結果如下：

- `200 / 80`, `cap=50`
  - `reached = 0.9536`
  - `exposure = 23.0600`
  - `t90 = 230.29`
  - `t95 = 361.4`
  - `reassignments = 41.12`

- `300 / 120`, `cap=70`
  - `reached = 0.9075`
  - `exposure = 37.6288`
  - `t90 = 327.25`
  - `t95 = None`
  - `reassignments = 102.33`

- `500 / 200`, `cap=100`
  - `reached = 0.8571`
  - `exposure = 32.6220`
  - `t90 = None`
  - `t95 = None`
  - `reassignments = 11774.2`

這說明：

- 單純 agent scaling 並不足以逼出真正壓力
- 一旦加入 shelter capacity，系統效能會顯著下降
- 高壓規模下的主要問題之一是大量 reassignment churn

## 十三、目前已完成的補強

更新日期：2026-03-23

針對 shelter capacity 所造成的 churn，目前已完成兩個補強：

1. `capacity-aware initial shelter assignment`
   - 初始分配時就考慮 shelter 容量

2. `taboo shelter memory`
   - shelter 拒收後，agent 暫時不再回選同一個 shelter

目前狀態：

- 兩個機制都已經完成實作
- `capacity-aware initial shelter assignment` 已證明對中等壓力場景有部分幫助
- `taboo shelter memory` 已完成新一輪驗證，但 aggregate KPI 沒有明顯改善

## 十四、interaction-aware 與 zone-level 後續方向

更新日期：2026-03-23

在 capacity scaling 之後，又補做了兩條延伸線：

1. `edge congestion-aware reranking`
2. `zone-level capacity-aware demand balancing`

目前判讀如下：

- `edge congestion-aware reranking`
  - 已完成第一版
  - 但在 capacity scenarios 下沒有帶來穩定、全面的 KPI 改善
  - 因此目前保留為 experimental branch

- `zone-level capacity-aware demand balancing`
  - 已完成第一版
  - zone assignment 不再只看最近 shelter
  - 會同時考慮：
    - 平均距離
    - 剩餘容量
    - demand / remaining 壓力
    - overload 懲罰
  - 這條線目前比純 reranking 更值得優先驗證

因此目前 scaling 線的下一步，不是再加更大的 agent 數量，而是：

1. 在 capacity scenario 下重跑 `zone assignment`
2. 再重跑 `zone route recommendation`
3. 檢查 zone-level primary / backup shelter 是否更合理
