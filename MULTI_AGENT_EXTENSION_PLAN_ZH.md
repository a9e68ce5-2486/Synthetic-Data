# Multi-Agent 擴充規劃

> 文件建立日期：2026-03-23
>
> 最近更新日期：2026-03-24
>
> 本文件對應 single-agent DRQN 主線完成後，如何逐步擴充到 multi-agent interaction-aware 系統。

## 一、目前判斷

目前不建議直接切到完整 multi-agent RL。

原因：

- single-agent DRQN 主線已經穩定
- easy、blocked、multi-seed、baseline comparison 都已完成
- 純數量 scaling 到 `500 / 200` 也沒有崩潰

真正暴露問題的點不是 routing core，而是：

- shelter capacity
- resource competition
- congestion
- reassignment churn

因此，下一步最合理的是：

- 先做 **interaction-aware shared-policy extension**
- 再決定是否需要 full MARL

## 二、分三層推進

### 第一層：interaction-aware single-agent / shared policy

這一層不改 DRQN 主體訓練架構，只補互動資訊與執行期修正。

優先項：

- local density / congestion
- shelter occupancy
- vehicle / shuttle availability
- queueing / edge slowdown

這一層的目標是：

- 保留目前已證明有效的 DRQN
- 讓同一個 shared policy 在多 agent 執行時更懂互動壓力

### 第二層：shared-policy multi-agent execution

在第一層基礎上，把 observation 與 reranking 做得更完整，例如：

- local crowd density features
- edge occupancy features
- shelter occupancy ratio
- mode availability

仍然維持：

- shared policy
- decentralized execution

### 第三層：真正 multi-agent RL

只有當前兩層仍不足以處理：

- 強烈壅塞
- shelter competition
- multimodal coordination

才進一步考慮：

- centralized training / decentralized execution
- QMIX / VDN 類方法
- 顯式 joint policy learning

## 三、目前已完成的第一個 interaction-aware feature

更新日期：2026-03-23

已完成：

- `edge congestion-aware reranking` 第一版

修改檔案：

- `config.py`
- `batch_runner.py`

新增設定：

- `EVAC_INTERACTION_DENSITY_ENABLED`
- `EVAC_INTERACTION_EDGE_PENALTY`
- `EVAC_INTERACTION_NODE_PENALTY`
- `EVAC_INTERACTION_NEARBY_PENALTY`
- `EVAC_INTERACTION_RADIUS_M`

目前邏輯：

- 不修改 DRQN checkpoint 的 observation 維度
- 不要求重訓主線模型
- 在多 agent 執行時，對每個候選鄰居額外計算：
  - 目標 edge 當前 occupancy
  - 目標 node 當前 occupancy
  - 該 edge 中點附近半徑內的 agent density
  - 同一 decision step 中已被其他 agent 選中的 edge 次數
- 再把這些互動壓力轉成 congestion cost，對 Q-value 做 reranking

這樣做的目的：

- 讓 shared DRQN policy 在 execution 時避開局部擁塞
- 降低 agent herd 到同一節點或同一路段的風險
- 不破壞目前已證明有效的單 agent 主線

目前驗證結論：

- 這一版 `edge congestion-aware reranking` 比先前的 `node density penalty` 更合理
- 但在 capacity scenarios 下，仍未形成穩定、全面的 KPI 改善
- 因此目前應視為 `experimental branch`，不宜直接升為主線預設

## 四、為什麼先做 reranking，不直接改 observation 維度

因為目前 final DRQN checkpoint 已經穩定，直接改 observation 維度會造成：

- checkpoint 不相容
- 必須重訓
- 新變因一次增加過多

先做 execution-time reranking 的好處：

- 可以快速驗證 interaction feature 是否值得保留
- 可以和目前 final checkpoint 直接相容
- 失敗成本低

如果後續證明有效，再把同樣的互動訊號正式併進訓練 observation。

## 五、建議的下一步順序

1. 優先做 `zone-level capacity-aware demand balancing`
2. 再補 `hard capacity-respecting assignment`
3. 在 zone route recommendation 中加入：
   - `route-feasibility filter`
   - `backup quality threshold`
   - `primary route-quality-aware recommendation`
4. 用 capacity scenario 重跑：
   - `zone_assignment`
   - `zone_route_recommendation`
5. 再比較：
   - zone 分配是否更平均
   - primary / backup shelter 是否更合理
   - 是否能降低高壓場景下的 shelter competition
6. 若這條線仍不足，再考慮第二個 interaction-aware feature：
   - `shelter occupancy aware reranking`
7. 只有當 assignment 與 reranking 都不足時，再考慮正式重訓 interaction-aware DRQN

## 六、目前新進度

截至目前，zone-level planning 已完成：

- `capacity-aware demand balancing`
- `hard capacity-respecting assignment`
- `route-feasibility filter`
- `backup quality threshold`
- `primary route-quality-aware recommendation`

這代表目前的 enterprise-oriented extension 已經不只是：

- 把 zone 指到最近或最空的 shelter

而是已經能同時考慮：

- 容量是否合法
- 主 / 備援路線是否可達
- 備援品質是否明顯過差
- 在 recommendation 層上，是否有更好的主路線選擇

目前仍未完成的部分是：

- 將 `recommended_*` 進一步回饋到 assignment core
- 讓 primary assignment 直接內生 route quality，而不只是後處理 rerank
- building / department 真實區塊化
- 將 zone recommendation 直接對接到更高層的區域 SOP
- `edge-level queueing / slowdown`
- `shelter occupancy aware routing`
- `vehicle / shuttle competition`
- 顯式 group-level coordination

目前已新增：

- `backup_status`
- `recommended_backup_status`
- `backup_weak`
- `zones_with_weak_backup`
- `*_management_summary.txt`

因此，這條線現在已不只是研究用 JSON / Markdown，而是開始有管理端可直接閱讀的輸出摘要。

## 七、目前仍缺的 multi-agent interaction

目前雖然已經有：

- shelter capacity
- zone assignment
- route recommendation
- weak backup detection
- interaction-aware reranking 試驗版

但嚴格來說，這還不是完整的 multi-agent interaction modeling。

目前最明確還缺的是：

1. `edge-level queueing / slowdown`
2. `shelter occupancy aware routing`
3. `vehicle / shuttle competition`
4. `explicit coordinated multi-agent policy`

因此目前的判斷仍是：

- 你現在缺的是 `multi-agent interaction mechanism`
- 不是立刻缺一個 `full MARL algorithm`

## 八、目前定位

目前這條線不是：

- full multi-agent RL

而是：

- shared-policy
- interaction-aware
- enterprise-oriented evacuation execution layer

這是比較穩、也比較符合目前專案節奏的做法。
