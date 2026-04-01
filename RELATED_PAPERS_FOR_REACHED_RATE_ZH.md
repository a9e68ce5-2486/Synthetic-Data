# 可用來提升 Reached Rate 的相關論文整理

> 文件建立日期：2026-03-21
>  
> 最近更新日期：2026-03-31
>
> 本文件整理與評估的文獻方向，對應目前 2026-03-21 至 2026-03-31 的主線設計決策

這份文件整理除了 BEAG 之外，還有哪些論文中的概念與目前專案相近，並且有機會實際應用到目前的 DRQN graph-based evacuation 系統中，以進一步提高 reached rate。

重點不是單純列文獻，而是回答兩個問題：

- 這篇論文的核心概念是什麼
- 這個概念能怎麼套到目前專案

## 1. Hindsight Experience Replay (HER)

### 核心概念

HER 的想法是：

- 很多 episode 雖然沒有達成最終目標
- 但其實途中到過一些中間狀態
- 可以把這些中間狀態「重新當成目標」，讓失敗經驗也變成可學習樣本

換句話說，HER 是在解 sparse reward 問題。

### 為什麼和目前專案有關

你現在的任務本質上就是：

- 從起點走到 shelter
- 中途可能經過 checkpoint / subgoal

這代表很多失敗 episode 並不是完全沒價值，因為 agent 可能：

- 走對了前半段
- 到過一些 checkpoint
- 接近過 shelter 但最後失敗

### 可以怎麼套到目前專案

最自然的做法是：

- 把沒有到 shelter 的 episode
- 改寫成「成功到達某個 checkpoint / 某個中間節點」

例如：

- 如果 agent 沒到最終 shelter
- 但成功到過第 2 個 checkpoint
- 那這段經驗可以重新標記成對應 checkpoint 的成功樣本

### 預期效益

- 提高失敗軌跡的利用率
- 增加 reward signal 密度
- 對長路徑與 sparse reward 問題特別有幫助

### 參考

- Hindsight Experience Replay
- https://arxiv.org/abs/1707.01495

## 2. Teacher-Student Curriculum Learning (TSCL)

### 核心概念

TSCL 的想法是：

- curriculum 不一定要手動固定
- 可以根據模型目前的學習進度，自動調整下一步該練什麼難度

也就是讓系統自己決定：

- 現在應該多練簡單一點的起點
- 還是可以開始增加更難的起點

### 為什麼和目前專案有關

你現在已經證明 curriculum 很重要，但問題也很明顯：

- 放太快，模型會掉下來
- 放太慢，又可能浪費訓練資源

這表示現在的 curriculum 還是偏手動。

### 可以怎麼套到目前專案

可以讓系統依照最近一段時間的：

- moving average reached rate
- moving average return
- 不同距離區間的成功率

自動調整：

- `curriculum_end_dist`
- `coverage_ratio`
- 或 episode 起點的抽樣分布

例如：

- 如果最近在 `0.4 ~ 0.5` coverage 區間成功率很高
- 就往更高 coverage 擴張
- 如果擴張後成功率掉太多
- 就暫停或退回較容易區間

### 預期效益

- curriculum 會比固定 schedule 更合理
- 有助於減少後段突然崩掉
- 更有機會把 easy mode reached rate 再往上推

### 參考

- Teacher-Student Curriculum Learning
- OpenAI article: https://openai.com/index/teacher-student-curriculum-learning/

## 3. Go-Explore

### 核心概念

Go-Explore 的重要想法是：

- 先「回到有潛力的狀態」
- 再從那裡繼續往外探索

它特別強調不要每次都從頭亂試，而是保存值得繼續探索的狀態。

### 為什麼和目前專案有關

你現在其實已經有一些類似概念：

- checkpoint
- frontier bonus
- revisit penalty

但目前還沒有真正做到：

- 保存高價值 frontier states
- 從這些狀態重新啟動探索

### 可以怎麼套到目前專案

可以保存以下狀態作為「可回訪 frontier」：

- 到過某個關鍵 checkpoint 的狀態
- 成功進入某個困難區域但還沒到 shelter 的狀態
- 接近 shelter 但最後失敗的狀態

之後訓練時，部分 episodes 不一定從隨機 start 開始，而是：

- 從這些已知有潛力的 frontier states 開始

### 預期效益

- 減少從頭探索的浪費
- 增加對困難區域的學習密度
- 對降低局部停滯和長距離失敗很有幫助

### 參考

- Go-Explore: a New Approach for Hard-Exploration Problems
- https://arxiv.org/abs/1901.10995

## 4. D* Lite

### 核心概念

D* Lite 是動態路徑規劃經典方法之一。

它的重點不是一般靜態最短路，而是：

- 當環境中某些邊失效
- 不要每次都從頭完全重算
- 而是做增量式 replanning

### 為什麼和目前專案有關

你現在已經加了：

- blocked-aware replanning

但目前做法仍偏向：

- 遇到 blocked edge 就重新規劃一次 checkpoint 路徑

這雖然有效，但還不算真正的增量式 planner。

### 可以怎麼套到目前專案

可以把目前的 replanning 再升級成：

- 更快更新被封路後的可行路徑
- 更穩定處理 snow blockage / earthquake blockage
- 甚至把 planner 的結果當作 DRQN 的輔助訊號

例如：

- D* Lite 提供 blocked-aware shortest path estimate
- DRQN 再在此基礎上決定局部策略

### 預期效益

- 提高 full hazard setting 下的穩定性
- 降低 agent 因封路而卡住的機率
- 對你之後做 final blocked model 特別有用

### 參考

- D* Lite
- https://www.ri.cmu.edu/publications/d-lite/

## 5. HIRO

### 核心概念

HIRO 是一種 hierarchical reinforcement learning 方法。

它把策略分成：

- 高層：產生目標或 subgoal
- 低層：執行具體動作去達成這些 subgoal

### 為什麼和目前專案有關

你現在雖然已經有 subgoal / checkpoint learning，但它還是：

- 規則式產生 checkpoint
- 不是學出來的高層目標

### 可以怎麼套到目前專案

未來可以把現在的 checkpoint 機制升級成：

- 高層 policy 動態選擇下一個 subgoal node
- 低層 DRQN 負責從當前位置到該 subgoal

### 預期效益

- 比固定 checkpoint 更有彈性
- 適合動態 blocked environment
- 讓 long-horizon routing 更容易學

### 參考

- HIRO: Hierarchical Reinforcement Learning with Off-Policy Correction
- https://arxiv.org/abs/1805.08296

## 6. Option-Critic Architecture

### 核心概念

Option-Critic 的重點是學會「temporally extended actions」，也就是：

- 不只是一步一步選 primitive action
- 而是學一個可以持續多步執行的 option

### 為什麼和目前專案有關

你目前在 graph 上每一步都重新選鄰居，這有時會造成：

- 局部抖動
- 繞圈
- 在分岔口附近來回改方向

### 可以怎麼套到目前專案

可以把某些局部行為變成 option，例如：

- 持續往某個 checkpoint 前進
- 持續脫離某個高風險區域
- 持續往某個子區域出口移動

### 預期效益

- 讓路徑行為更穩定
- 降低 graph 上的決策抖動
- 對動畫表現和最終到達率都有機會改善

### 參考

- The Option-Critic Architecture
- https://arxiv.org/abs/1609.05140

## 7. FeUdal Networks (FuN)

### 核心概念

FuN 也是一種 hierarchical RL。

它把 agent 拆成：

- Manager：決定抽象目標
- Worker：執行具體行動

### 為什麼和目前專案有關

它和 HIRO 一樣，都適合處理：

- 長路徑任務
- sparse reward
- 分層導航

### 可以怎麼套到目前專案

如果未來你不想只用規則式 checkpoint，可以考慮：

- 用 manager 決定下一個中繼方向或區域
- worker 保持現在的局部 graph action 決策

### 預期效益

- 對長距離 shelter routing 有理論吸引力
- 但實作成本比 HIRO 還高

### 參考

- FeUdal Networks for Hierarchical Reinforcement Learning
- https://arxiv.org/abs/1703.01161

## 8. Random Network Distillation (RND) / Count-Based Exploration

### 核心概念

這類方法的共同點是：

- 對沒看過或少看過的狀態給 intrinsic reward
- 鼓勵 agent 主動探索 novelty

### 為什麼和目前專案有關

你現在已經有：

- frontier bonus
- revisit penalty

但目前還偏手工設計。

### 可以怎麼套到目前專案

可以把現在的 frontier/revisit 控制升級成：

- 依據 state novelty 自動給 intrinsic reward
- 而不是只靠簡單 visit count

### 預期效益

- 探索訊號更系統化
- 可能比手工 bonus 更穩定
- 特別適合需要覆蓋更多起點的情況

### 參考

- Random Network Distillation
- https://arxiv.org/abs/1810.12894

- Count-Based Exploration with Neural Density Models
- https://arxiv.org/abs/1703.01310

## 對目前專案的優先順序建議

如果目標是「以最小額外成本，提高目前系統的 reached rate」，優先順序我會建議如下：

### 第一優先

1. HER
2. TSCL
3. D* Lite style blocked replanning enhancement
4. **Domain Randomization（已實作，2026-03-29）**

這四個最接近你現在的痛點：

- sparse reward
- curriculum 如何放寬
- 封路後怎麼更穩定重規劃
- 訓練/評估 distribution mismatch

### 第二優先

5. Go-Explore
6. RND / count-based exploration

這兩個主要針對：

- 探索效率
- frontier state 利用
- 減少局部停滯

### 第三優先

7. HIRO
8. Option-Critic
9. FeUdal Networks

這三個更偏向模型架構升級。

它們理論上很強，但：

- 實作成本高
- 調參難度高
- 對目前專案來說不一定是最快提升 reached rate 的方法

## 實作進度（2026-03-31 更新）

| 方法 | 狀態 | 說明 |
|------|------|------|
| TSCL / Adaptive Curriculum | ✅ 已實作 | `adaptive_distance` curriculum mode |
| Failure-aware exploration | ✅ 已實作 | `failure_memory` 機制 |
| Subgoal / Checkpoint | ✅ 已實作 | `use_subgoals` 機制 |
| Dynamic step budget | ✅ 已實作 | `dynamic_step_budget` 機制，參數已修正（scale=0.35, min=400） |
| Blocked-aware replanning | ✅ 已實作 | `replan_on_block` 機制 |
| **Domain Randomization** | ✅ **已實作（2026-03-29）** | `--domain-rand` CLI flag，每 episode 隨機抽取 hazard 強度 |
| **Progressive severity curriculum** | ✅ **已實作（2026-03-29）** | `finetune_progressive_severity.sh`，4-stage 對應 light/moderate/severe/extreme |
| **Reachable-only spawn nodes** | ✅ **已實作（2026-03-31）** | `batch_runner.py` 新增 `_reachable_walk_nodes()` / `_reachable_drive_nodes()`，agent 只生成在可達 shelter 的節點上 |
| **Severity block_init calibration** | ✅ **已實作（2026-03-31）** | `scenario_loader.py` extreme blizzard `EVAC_BLOCK_INIT_PROB` 從 0.12 降至 0.07，消除孤立節點問題 |
| **Reachability analysis** | ✅ **已實作（2026-03-31）** | `analyze_reachability.py`：分類 UNREACHABLE / BUDGET_LIMITED / POLICY_SOLVABLE |
| HER | ❌ 尚未實作 | 修改 replay buffer，重標記失敗軌跡 |
| D* Lite | ❌ 尚未實作 | 增量式動態路徑規劃取代現有全量 replan |
| Go-Explore | ❌ 尚未實作 | 保存 frontier states 作為 episode 起點 |
| RND / Count-based | ❌ 尚未實作 | 取代手工 frontier_bonus |
| HIRO | ❌ 尚未實作 | 學習式 subgoal 取代規則式 checkpoint |
| Option-Critic | ❌ 尚未實作 | 時間延伸動作 |
| FeUdal Networks | ❌ 尚未實作 | Manager-Worker 層級結構 |

## 總結

除了 BEAG 之外，還有不少論文中的概念能幫助提高 reached rate，但不是每個都適合現在立刻做。

2026-03-29 針對「跨嚴重度效能降低」的訓練/評估 distribution mismatch 做出第一輪修正（domain randomization + progressive severity curriculum），但 sweep 結果顯示 v1 兩組新模型和舊模型的 reached_rate 幾乎相同，均停在約 0.69。

2026-03-31 完成更深層的 reachability 分析，確認真正根因：

- **Isolated node spawning（孤立節點生成）**：extreme severity 下 18.2% 的 agent 被生成在所有鄰居邊都被初始封路的節點，candidates=[]，policy 完全無法作用。
- **`EVAC_BLOCK_INIT_PROB` 設定過高**：extreme 設定 0.12 在 low-degree 節點密集的 OSM walk graph 上，導致大量孤立節點出現。

修正已實作：

- **`batch_runner.py`**：agent spawn 節點從全 walk/drive nodes 改為只使用 `nx.ancestors(unblocked_graph, shelter)` 的可達節點池，從根本消除孤立節點問題。
- **`scenario_loader.py`**：extreme blizzard `EVAC_BLOCK_INIT_PROB` 從 0.12 降至 0.07，對齊 earthquake extreme 設定，減少孤立節點的形成頻率。

目前正在執行 post-fix severity sweep 驗證中。

下一步最值得優先補的依然是：

- HER：提高失敗樣本利用率（尤其在高嚴重度下大量失敗 episode 浪費掉）
- D* Lite：讓 blocked 後的 replan 更快更準
- TSCL per severity：讓各嚴重度各有自己的 adaptive 難度控制

目前最務實的策略不是一次把所有想法都加進去，而是：

- 每次只引入一個真正對 reached rate 有幫助的機制
- 重訓
- 做 eval
- 再決定要不要保留
