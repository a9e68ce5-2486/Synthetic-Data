# Multi-Agent / Capacity / Congestion 相關文獻整理

> 文件建立日期：2026-03-23
>
> 最近更新日期：2026-03-23
>
> 本文件只整理和目前專案瓶頸直接相關的文獻：
> shelter capacity、congestion、zone assignment、shared-policy multi-agent execution。

## 一、目前問題對應

你目前已經證明：

- single-agent DRQN routing core 很穩
- 純數量 scaling 到 `500 / 200` 也沒有崩

真正開始掉 KPI 的地方是：

- shelter capacity
- reassignment churn
- congestion / interaction pressure

所以文獻重點不應該先放在「更大的 DRQN」，
而應該放在：

- shelter assignment
- capacity-aware planning
- congestion-aware routing
- shared-policy / group-level evacuation control

## 二、最相關文獻

### 1. Evacuation Simulation with Limited Capacity Sinks
ㄏㄠ
- Authors: Gunnar Flötteröd, Gregor Lämmel
- Year: 2010
- Core idea:
  - 在有限 shelter capacity 下，同時處理 shelter allocation 與 route assignment
  - 不是等人走到 shelter 才發現滿了，而是從更高層先處理 assignment
- 為什麼和目前問題最相關：
  - 你現在最大的問題就是 `shelter capacity` 與 `reassignment churn`
  - 這篇剛好直接處理 limited-capacity shelters
- 對你模型最值得借用的點：
  - `zone-level capacity-aware shelter assignment`
  - `route + shelter joint optimization`
  - `fair` 與 `system-optimal` assignment 的區分
- 你目前可對應的實作：
  - zone assignment
  - capacity-aware initial assignment
  - 後續應往 capacity-aware zone balancing 推進
- Sources:
  - https://infoscience.epfl.ch/record/152343
  - https://infoscience.epfl.ch/bitstreams/2f4a6802-cb87-4a55-99b9-ddac43886e05/download

### 2. Congestion-aware Route Selection in Automatic Evacuation Guiding

- Authors: Kasai et al.
- Year: 2017
- Core idea:
  - 把 congestion 明確納入 route selection
  - 將道路壅塞與 travel time 納入引導成本
- 為什麼和目前問題最相關：
  - 你剛剛驗證過，單純 node density penalty 不夠穩
  - 問題更接近 edge-level congestion，而不是單點人數
- 對你模型最值得借用的點：
  - `edge congestion cost`
  - `travel-time aware reranking`
  - `道路壓力` 比 `單點密度` 更有代表性
- 你目前可對應的實作：
  - 已把 interaction-aware feature 改成 edge-first congestion reranking
  - 下一步可以把 edge congestion 從 reranking 再往 route cost 推進
- Source:
  - https://link.springer.com/article/10.1186/s13638-017-0948-6

### 3. Multi-Agent Path Finding with Capacity Constraints

- Authors: Pavel Surynek, T. K. Satish Kumar, Sven Koenig
- Year: 2019
- Core idea:
  - 在 graph path finding 中明確加入 vertex / capacity constraints
  - 把 capacity 當成 hard constraint，而不是事後懲罰
- 為什麼和目前問題最相關：
  - 你的場景本質上也是 graph-based routing
  - `shelter capacity`、`node congestion`、`edge occupancy` 都可以借這個角度 formalize
- 對你模型最值得借用的點：
  - reservation / occupancy constraints
  - vertex / edge capacity modeling
  - 從 soft reranking 逐步升級到 semi-hard constraints
- 你目前可對應的實作：
  - shelter capacity 已加入
  - 之後可加入 edge capacity / queueing
- Source:
  - https://www.emergentmind.com/articles/1907.12648

### 4. An Evacuation Guidance Approach Based on Multi-agent Shared Q-learning

- Authors: Han et al.
- Year: 2025
- Core idea:
  - 不是每個 agent 學一套，而是 shared Q-learning
  - reward 同時考慮 distance、density、time、route frequency
- 為什麼和目前問題最相關：
  - 你現在也不適合直接進 full MARL
  - 更合理的是 shared-policy + interaction-aware execution
- 對你模型最值得借用的點：
  - shared-policy multi-agent execution
  - density-aware reward / scoring
  - zone / group level guidance
- 你目前可對應的實作：
  - shared DRQN policy
  - interaction-aware reranking
  - zone assignment + zone route recommendation
- Source:
  - https://doi.org/10.1016/j.engappai.2025.111587

### 5. Multi-agent Deep Reinforcement Learning for Group Intelligence in Emergency Evacuation

- Year: 2026
- Core idea:
  - centralized training / decentralized execution
  - perception-aware policies
  - 面向多 agent 緊急疏散
- 為什麼相關：
  - 這是如果你未來真的要進 full multi-agent RL，很合理的方向
- 為什麼不是現在第一優先：
  - 你目前 bottleneck 還是在 capacity / assignment / congestion modeling
  - 不是 DRQN 架構能力不足
- 對你模型最值得借用的點：
  - CTDE
  - perception-aware local observation
  - group-level coordination
- Source:
  - https://www.sciencedirect.com/science/article/pii/S0952197625035201

## 三、對目前模型的直接建議

按照目前結果，最值得先做的不是 full MARL，而是：

1. `zone-level capacity-aware shelter assignment`
2. `edge congestion cost`
3. `shelter occupancy aware reranking`

這三件事的優先順序都高於：

- 重訓更大的 DRQN
- 直接切到完整 multi-agent RL

## 四、已經對應到你目前程式的部分

目前已經做的：

- `capacity-aware initial shelter assignment`
- `zone assignment`
- `zone route recommendation`
- `interaction-aware reranking`

目前最接近文獻建議、且仍值得繼續加強的部分：

- 把 interaction feature 從粗的 density penalty，進一步改成更穩定的 edge congestion cost
- 把 zone assignment 從「距離最近」升級成「容量平衡 + demand balancing」

## 五、目前最務實的下一步

1. 在 capacity scenario 下重新驗證 edge congestion-aware reranking
2. 若效果仍不穩，優先改做：
   - zone-level capacity-aware balancing
3. 只有當這兩條都做完還不夠，再考慮：
   - shared-policy multi-agent retraining
   - CTDE / full MARL

## 六、一句話總結

根據目前的實驗結果與文獻，下一步最值得做的是：

- 先把 `capacity-aware assignment + congestion-aware routing` 做扎實

而不是：

- 立刻把整個系統改成 full multi-agent RL
