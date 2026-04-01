# BEAG 啟發機制說明

> 文件建立日期：2026-03-21
>  
> 最近更新日期：2026-03-22
>  
> 內容對應的主要實作更新區間：2026-03-21 至 2026-03-22

這份文件整理目前 DRQN 訓練流程中已加入的幾個 BEAG-inspired 機制，說明它們是什麼、在系統裡扮演什麼角色，以及為什麼需要加入。

整體目的很一致：

- 單純依靠 DRQN 從 sparse reward 自己學會「在真實校園路網中避開災害並到達 shelter」太困難
- 因此需要額外加入一些結構化機制，幫助探索、穩定學習、降低繞圈與卡住的機率

## 1. Failure-Aware Exploration

### 是什麼

Failure-aware exploration 是一種「記住失敗邊」的探索機制。

當 agent 嘗試某一條邊 `(u, v)` 時，如果發生以下情況：

- 邊被封住
- 動作無效
- 反覆嘗試相同失敗路徑

系統會把這條邊記錄進 failure memory，並在後續一段時間內：

- 降低再次選到它的機率
- 或直接暫時視為不可用

### 為什麼要加

如果沒有這個機制，agent 很容易反覆撞上同樣的錯誤，特別是在：

- blocked edge 附近
- 局部死胡同
- 被災害破壞後的路網區域

這會導致：

- 探索效率很差
- 訓練資料充滿重複失敗
- 學習速度變慢

加入 failure-aware exploration 後，agent 可以更快放棄明顯不好的局部選擇，把探索資源放到更有希望的路徑上。

## 2. Subgoal / Checkpoint Learning

### 是什麼

Subgoal / checkpoint learning 是把「從起點走到最終 shelter」這個長任務切成多個中繼目標。

具體做法是：

- 先計算起點到 shelter 的路徑
- 再每隔一段距離或若干個節點切出一個 checkpoint
- agent 不只學最終目標，也學會逐段到達這些中繼點

### 為什麼要加

如果只把 shelter 當唯一目標，對 DRQN 來說會有兩個問題：

- 路徑太長，回饋太 sparse
- 很難知道自己是否在往正確方向前進

checkpoint 的作用是把長路徑任務拆小，讓 agent 有更密集的正向訊號：

- 到達 checkpoint 代表方向對了
- 接下來只需要專注下一段路徑

這能大幅降低長距離 routing 任務的學習難度。

## 3. Curriculum

### 是什麼

Curriculum 是一種由易到難的訓練策略。

不是一開始就讓 agent 面對所有起點，而是：

- 先從較近、較簡單的起點開始
- 再逐步擴大到更遠、更難的起點

目前系統裡用過兩種概念：

- distance-based curriculum
- coverage-based curriculum

### 為什麼要加

如果一開始就把所有困難案例一起丟給 DRQN，常見結果是：

- 早期幾乎全失敗
- Q-value 很難形成穩定結構
- 訓練直接卡住

curriculum 的作用是讓模型先學基本導航能力，再慢慢學更複雜的起點分佈。

在目前專案裡，easy pretrain 能夠成功，curriculum 是關鍵因素之一。

## 4. Dynamic Step Budget

### 是什麼

Dynamic step budget 是讓每個 episode 的 `max_steps` 根據起點到 shelter 的距離動態調整。

也就是：

- 起點離 shelter 近，允許步數可以少一點
- 起點離 shelter 遠，允許步數就要多一點

### 為什麼要加

如果所有 episode 都用相同固定步數，例如都只給 100 步，會有明顯問題：

- 近距離案例很寬鬆
- 遠距離案例可能根本來不及完成，就被強制判失敗

這會讓訓練訊號失真，因為遠距離失敗不一定是策略差，而可能只是時間上限不合理。

加入 dynamic step budget 後，不同難度案例會有比較公平的完成條件，也更符合真實任務的需求。

## 5. Blocked-Aware Replanning

### 是什麼

Blocked-aware replanning 是讓 agent 在遇到封路或原路徑失效時，能夠重新規劃中繼路徑與目標。

目前做法是：

- 原本會先生成一條 checkpoint 路徑
- 但如果中途某條路被 snow blockage 或其他災害封住
- 系統會從 agent 當前位置重新規劃到 shelter 的 checkpoint 路徑

### 為什麼要加

如果沒有這個機制，agent 可能會一直朝著舊 target 前進，但那條路其實已經不可達。

這代表：

- subgoal 雖然存在，但變成過期資訊
- agent 會一直卡在已失效的路徑規劃上

在靜態環境中這個問題不明顯，但在：

- 雪造成封路
- 地震造成邊失效
- 複合災害動態變化

這些情境下，不重新規劃就很難維持路徑策略有效性。

## 6. Frontier / Revisit Control

### 是什麼

Frontier / revisit control 是一種抑制局部繞圈、鼓勵探索新節點的控制機制。

目前包含兩部分：

- `frontier_bonus`
  - 第一次到達新節點時給額外獎勵
- `revisit_penalty`
  - 如果反覆回到已經訪問過的節點，給額外懲罰

此外，候選鄰居排序也會考慮：

- 對目前 target 的 graph progress 是否更好
- 該節點是否已經被重複拜訪太多次

### 為什麼要加

在 graph routing 問題裡，agent 常見的失敗模式不是完全不動，而是：

- 在局部區域來回震盪
- 進入小圈圈後一直繞
- 知道目標大方向，但無法真正走出去

frontier / revisit control 的目的就是：

- 鼓勵 agent 往新的、有希望的區域探索
- 降低重複回到舊節點的傾向

這對提升 reached rate 很重要，尤其是在真實路網而不是規則 grid 的情況下。

## 總結

這六個機制可以視為目前 DRQN 系統中的結構化輔助模組。它們的共同目標不是取代 DRQN，而是讓 DRQN 更容易學會真正有用的 shelter-routing 行為。

它們分別處理不同層面的問題：

- `failure-aware exploration`
  - 減少重複踩同樣的錯誤
- `subgoal / checkpoint learning`
  - 把長路徑拆成可學的小段
- `curriculum`
  - 讓模型由易到難學習
- `dynamic step budget`
  - 讓不同距離案例有合理步數上限
- `blocked-aware replanning`
  - 讓模型能在封路後重新找路
- `frontier / revisit control`
  - 降低繞圈與局部停滯

對目前專案而言，這些機制的角色是：

- 提升 reached rate
- 改善訓練穩定性
- 讓 DRQN 更適合真實校園路網與災害情境

但也要注意，這些機制雖然受到 BEAG 思想啟發，並不等於完整重現原始 BEAG 演算法。它們是依照本專案的 OSM graph evacuation 任務做過調整後的版本。
