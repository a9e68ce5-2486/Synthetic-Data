# LLM Agent 擴充：文獻整理與實作規劃

> 文件建立日期：2026-03-31
>
> 

---

## 一、為什麼需要加入 LLM Agent

目前系統的三個層面問題：

1. **所有 agent 行為參數完全相同**：速度、觀察誤差、決策邏輯對所有人一致，不符合真實人群的異質性（老人、行動不便者、學生、教職員反應完全不同）
2. **Zone assignment 是純演算法決策**：沒有語意推理能力，無法解釋「為什麼這個 zone 要去這個 shelter」

**解法**：三層架構

```
Layer 1 (LLM):  Human Behavior Profiling   → 生成 diverse agent profiles
Layer 2 (LLM):  Zone Coordinator Agent     → tool-using LLM 做 zone-level 決策
Layer 3 (DRQN): Navigation                 → 每個 agent 的逐步圖上路徑決策
```

---

## 二、相關論文整理

### 分類 A：LLM Agent 用於疏散模擬

#### 1. LLM Agents for Fire Evacuation Simulation

- **作者**：Pei Dang, Jun Zhu, Weilian Li, Yakun Xie, Heng Zhang
- **期刊**：*Safety Science*, Volume 191, Elsevier, 2025
- **DOI**：10.1016/j.ssci.2025.001602

**核心貢獻**：

用 GPT-4、ERNIE-Bot 4.0、Llama-2-70B、ChatGLM2-6B 整合 Cellular Automata，模擬 LiDAR 重建的購物中心火災疏散。每個 agent 有個人化記憶、認知與 LLM 決策。

關鍵發現：
- 更大的 LLM 產生更一致、更有效率的疏散策略
- Agent 背景（persona）與溝通行為對團體疏散結果影響顯著
- 不同 LLM 的行為差異可量化

**對本專案的啟示**：這就是「用 LLM 做 human behavior modeling」的最直接前人研究。可用來 justify Layer 1 的設計。

---

#### 2. FLARE: Wildfire Evacuation Decision Prediction with Behavioral Theory-Informed LLMs

- **作者**：Ruxiao Chen, Chenguang Wang, Yuran Sun, Xilei Zhao, Susu Xu
- **會議**：ACL 2025（Long Paper）
- **arXiv**：2502.17701

**核心貢獻**：

結合 LLM + 行為理論（Protection Motivation Theory）+ Chain-of-Thought + memory-based RL，預測個人在野火中是否疏散。

關鍵發現：
- 比傳統理論驅動行為模型平均改善 **+20.47%**
- 跨事件泛化能力強（在不同野火事件間 transfer）
- 行為理論 + LLM 的組合優於單純 LLM

**對本專案的啟示**：最接近的架構對標。LLM + RL 混合做疏散決策預測，和本專案的 LLM behavior profiling + DRQN navigation 的分工邏輯相同。

---

#### 3. What Makes LLM Agent Simulations Useful for Policy Practice? An Iterative Design Study in Emergency Preparedness

- **作者**：Yuxuan Li, Sauvik Das, Hirokazu Shirado
- **年份**：2025
- **arXiv**：2509.21868

**核心貢獻**：

與大學緊急應變團隊合作一年，把 LLM agent 模擬擴展到 **13,000 agents** 模擬校園緊急疏散。

三個設計原則：
1. 用可驗證情境建立使用者信任
2. 用初步模擬結果挖掘隱性領域知識
3. 把模擬精煉與政策實施視為相互強化的過程

**對本專案的啟示**：最接近你的應用場景（大學校園 + 緊急疏散 + policy tool）。13k agents 的規模驗證了 LLM 模擬的可行性，三個設計原則可直接用在最終報告的 system design 敘事中。

---

### 分類 B：LLM 社會模擬基礎架構

#### 4. Generative Agents: Interactive Simulacra of Human Behavior

- **作者**：Joon Sung Park, Joseph O'Brien, Carrie Jun Cai, Meredith Ringel Morris, Percy Liang, Michael S. Bernstein
- **會議**：ACM UIST 2023（Best Paper Award）

**核心貢獻**：

提出 generative agent 架構：
- **Memory stream**：natural language episodic 記憶
- **Reflection**：定期合成高層次洞察
- **Retrieval**：動態取回相關記憶支持規劃

25 個 agent 在 Sims-like 環境中自主傳播派對邀請、形成社交關係、協調活動。

**對本專案的啟示**：Layer 1 LLM behavior profiling 的理論基礎。Memory-reflect-retrieve-plan 的 loop 可用來模擬疏散者如何根據動態資訊（shelter 滿了、某條路封了）更新決策。

---

#### 5. Generative Agent Simulations of 1,000 People

- **作者**：Joon Sung Park 等（Stanford HAI）
- **年份**：2024
- **arXiv**：2411.10109

**核心貢獻**：

訪談 1,052 名真實人士，建立 digital twin。LLM agents 複製真實答案的準確率達到真人兩週後記憶準確率的 **85%**。在種族與意識形態群體間的準確性差距也比純人口屬性條件化方法更小。

**對本專案的啟示**：驗證了「用 LLM persona 代表真實多樣人口」的可行性，直接支持用 LLM 生成 faculty / student / mobility-impaired 等 persona 的做法。

---

#### 6. AgentSociety: Large-Scale Simulation of LLM-Driven Generative Agents

- **作者**：Jinghua Piao 等（Tsinghua University）
- **年份**：2025
- **arXiv**：2502.08691

**核心貢獻**：

城市規模 **10,000+ LLM agents**，包含顆粒情感、需求、動機與認知狀態。其中有**颶風衝擊實驗**，模擬外部災害對社會行為的影響，結果與真實資料對齊。

**對本專案的啟示**：颶風衝擊實驗直接對應災害疏散情境，驗證大規模 LLM agent 在 disaster scenario 的可行性。

---

#### 7. Large Language Models Empowered Agent-Based Modeling and Simulation: A Survey

- **作者**：Chen Gao, Xiaochong Lan, Nian Li 等（Tsinghua）
- **期刊**：*Humanities and Social Sciences Communications*（Nature Portfolio, 2024）
- **DOI**：10.1038/s41599-024-03611-3

**核心貢獻**：

整合傳統 ABM 與 LLM agent 的系統性綜述，涵蓋：
- Persona 設計方法
- 記憶模組架構
- 規劃模組架構
- Agent 社會協作策略

**對本專案的啟示**：直接橋接傳統疏散 ABM 與 LLM agent 設計，提供 Layer 1 behavior profiling 的系統性方法論基礎。

---

### 分類 C：LLM + RL 混合架構

#### 8. SayCan: Do As I Can, Not As I Say

- **作者**：Michael Ahn 等（Google Robotics，42 名作者）
- **會議**：CoRL 2022
- **arXiv**：2204.01691

**核心貢獻**：

Seminal「LLM 作 planner，RL 作 executor」論文。LLM 提供高層語意計畫，pre-trained RL value function（affordance）評估哪些行動在當前狀態可行。長距離家務任務達成率 70%。

**對本專案的啟示**：Layer 2（LLM zone coordinator）+ Layer 3（DRQN navigation）分層架構的理論基礎。LLM 決定「去哪個 shelter」，DRQN 決定「怎麼走」。

---

#### 9. Plan-Seq-Learn: Language Model Guided RL for Long Horizon Tasks

- **作者**：Murtaza Dalal, Tarun Chiruvolu, Devendra Chaplot, Ruslan Salakhutdinov
- **會議**：ICLR 2024

**核心貢獻**：

LLM 生成高層 subgoal 序列 → motion planner 執行 subgoal 間的轉移 → RL 在線學習低層控制。不需要預先定義 skill library。25+ 任務成功率 85%+。

**對本專案的啟示**：比 SayCan 更靈活。LLM 產生 zone-level waypoints / shelter sequence，DRQN 負責每步圖上決策，不需要預先定義每個「技能」。

---

#### 10. ReAct: Synergizing Reasoning and Acting in Language Models

- **作者**：Shunyu Yao, Jeffrey Zhao, Dian Yu 等
- **會議**：ICLR 2023
- **arXiv**：2210.03629

**核心貢獻**：

LLM 同步做 reasoning trace + tool call 的標準 agent loop。在 HotPotQA、ALFWorld、WebShop 等多個 benchmark 上優於純 RL 或純模仿學習，只需 1-2 個 in-context example。

**對本專案的啟示**：Layer 2 LLM zone coordinator 的實作模板。Reasoning → 查詢工具（shelter status、road conditions）→ 輸出分配決策，就是 ReAct loop。

---

#### 11. Inner Monologue: Embodied Reasoning through Planning with Language Models

- **作者**：Wenlong Huang 等（Google）
- **會議**：CoRL 2022
- **arXiv**：2207.05608

**核心貢獻**：

LLM 接收自然語言環境反饋（成功偵測器、場景描述、人工修正），形成「closed-loop inner monologue」並動態重規劃。不需要 fine-tuning。

**對本專案的啟示**：Zone coordinator 接收即時模擬狀態更新（壅塞、shelter 容量、封路）並動態修正分區指派，就是這篇的疏散情境版本。

---

### 分類 D：合成人口與 Persona 建模

#### 12. Large Language Models as Urban Residents: An LLM Agent Framework for Personal Mobility Generation

- **作者**：Jiawei Wang, Renhe Jiang, Chuang Yang 等
- **會議**：NeurIPS 2024
- **arXiv**：2402.14744

**核心貢獻**：

第一個用 LLM 生成有人口屬性（年齡、收入、職業）的合成人口並模擬個人化每日移動模式的框架。用 self-consistency calibration + RAG 對齊真實 travel survey 資料。

**對本專案的啟示**：Layer 1 合成人口生成的具體方法論。人口屬性（年齡、職位、行動能力）→ LLM → 行為參數（速度、compliance、panic level）的 pipeline 可直接對應。

---

### 分類 E：LLM Agent 評估與災害應用綜述

#### 13. Harnessing Large Language Models for Disaster Management: A Survey

- **作者**：Zhenyu Lei 等
- **年份**：2025
- **arXiv**：2501.06932

**核心貢獻**：

第一個針對完整自然災害管理生命週期（災前、應變、復原）的 LLM 應用系統性綜述，分析 70+ 篇研究（2020-2024）。

**對本專案的啟示**：Related work 文獻脈絡建立用，提供「LLM 在災害管理哪些環節已有應用」的全貌，用來定位你的系統在哪個缺口。

---

#### 14. AgentBench: Evaluating LLMs as Agents

- **作者**：Xiao Liu 等（Tsinghua / THUDM）
- **會議**：ICLR 2024
- **arXiv**：2308.03688

**核心貢獻**：

8 個環境的 LLM agent benchmark（OS、資料庫、知識圖、遊戲、網頁），評估 29 個 LLM。主要瓶頸是長期推理能力與指令跟隨能力。

**對本專案的啟示**：評估 LLM zone coordinator 效能時，可參考 AgentBench 的評估方法論。

---

#### 15. From Individual to Society: A Survey on Social Simulation Driven by LLM-based Agents

- **作者**：Xinyi Mou 等（Fudan）
- **年份**：2024
- **arXiv**：2412.03563

**核心貢獻**：

最完整的 LLM 社會模擬綜述，三層分類：Individual Simulation、Scenario Simulation、Society Simulation。疏散屬於「Scenario Simulation」（多 agent 在定義情境中協作）。

---

## 三、本專案的實作計畫

### Layer 1：LLM Human Behavior Profiler

**目標**：用 LLM 將 persona 描述轉換成定量行為參數，讓模擬中出現真實的人群異質性

**Persona 設計（5 類）**：

| Persona | 描述 | 預期行為特徵 |
|---------|------|------------|
| senior_faculty | 60歲以上教授，行動受限，沉著冷靜 | 速度↓、compliance↑、panic↓ |
| young_student | 20歲學生，高行動力，中等恐慌傾向 | 速度↑、compliance↓、panic↑ |
| staff_admin | 行政人員，熟悉校園，有疏散訓練 | 速度=、compliance↑↑、知道 shelter 位置 |
| mobility_impaired | 輪椅使用者或視障，依賴輔助 | 速度↓↓、需要特定路線 |
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

---

### Layer 2：LLM Zone Coordinator Agent

**目標**：用 tool-using LLM（ReAct loop）做 zone-level shelter 分配，可與現有 DRQN-based zone recommendation 直接比較

**LLM 可用工具**：

```python
get_shelter_status()     # 各 shelter 剩餘容量、目前 occupancy
get_zone_population()    # 各 zone 人數、persona 分布
get_road_conditions()    # 封路狀況、嚴重度等級
get_distance_matrix()    # zone centroid 到各 shelter 的距離
assign_zone_to_shelter() # 輸出分配決策
```

**實作檔案**：`llm_zone_coordinator.py`

**對比實驗**：LLM coordinator vs DRQN-based zone recommendation，比較 reached_rate、shelter capacity utilization、weak backup zone 數量

---

### Layer 3：DRQN Navigation（現有）

保持現有 DRQN 系統，接收 Layer 1 的 persona 參數作為 agent 初始化輸入，接收 Layer 2 的 zone assignment 作為 target shelter 設定。

---

## 四、對論文的敘事貢獻

**Title framing**（建議）：

> A Three-Layer LLM-DRQN Architecture for Campus Evacuation Planning: LLM-Driven Human Behavior Modeling, Zone Coordination, and Graph-Based Navigation

**三層的貢獻分工**：

| 層 | 方法 | 解決什麼問題 |
|----|------|------------|
| Layer 1 | LLM Behavior Profiling | 人群異質性建模（年齡、能力、恐慌傾向） |
| Layer 2 | LLM Zone Coordinator (ReAct) | 語意驅動的 zone-level shelter 分配 |
| Layer 3 | DRQN on OSM graph | 部分觀察環境下的個體逐步導航 |

**Related work 定位**：

- 相較於 Dang et al. (2025)：同樣用 LLM 做 evacuation behavior modeling，但加入了 graph-based DRQN navigation 和 zone-level coordination
- 相較於 FLARE (Chen et al., 2025)：同樣是 LLM + RL 混合，但針對 campus 場景並加入 multi-zone coordination
- 相較於 Li et al. (2025)：同樣針對 campus emergency，但加入了 LLM-profiled diverse agents + DRQN routing

---

## 五、實作進度（2026-04-01）

| 步驟 | 工作 | 狀態 |
|------|------|------|
| 1 | 設計 5 個 persona，用 Groq (Llama 3.3 70B) 生成行為參數 | ✅ 完成 |
| 2 | `llm_behavior_profiler.py` + `agent_profiles.json` | ✅ 完成 |
| 3 | `agents/base_agent.py` 加入 persona 欄位 | ✅ 完成 |
| 4 | `agents/ped_agent.py` 速度乘以 `speed_multiplier` | ✅ 完成 |
| 5 | `batch_runner.py` 自動讀 profiles、依 role 分配 persona | ✅ 完成 |
| 6 | 5-persona severity sweep 對比 uniform agents | ✅ 完成 |
| 7 | 擴充至 20 persona（4 role 類別，覆蓋大學真實人口組成） | ✅ 完成 |
| 8 | 20-persona severity sweep（對比 5-persona 與 uniform） | 🔄 進行中 |
| 9 | `llm_zone_coordinator.py`（ReAct loop + 4 工具） | ⬜ 待做 |
| 10 | LLM coordinator vs DRQN zone recommendation 對比 | ⬜ 待做 |
| 11 | 整合三層 end-to-end pipeline | ⬜ 待做 |

### 5-persona vs Uniform 比較結果

| Severity | Uniform | LLM-profiled (5) | Δ reached |
|----------|---------|-----------------|-----------|
| light    | 0.8482  | 0.8346          | −0.0136   |
| moderate | 0.7918  | 0.7736          | −0.0182   |
| severe   | 0.7364  | 0.7336          | −0.0028   |
| extreme  | 0.7164  | 0.7118          | −0.0046   |

LLM-profiled 比 uniform 低約 1-2%，這是預期中的正確結果——反映真實人群的異質性。

### 20-persona 擴充（Llama 3.3 70B via Groq）

**4 個 Role 類別與人口比例：**

| Role | 比例 | Personas |
|------|------|---------|
| student | 60% | young_student, freshman_student, graduate_student, international_student, student_athlete, student_with_anxiety, part_time_student |
| faculty | 15% | senior_faculty, junior_faculty, adjunct_instructor |
| staff | 20% | staff_admin, facilities_staff, campus_security, healthcare_staff, research_scientist, it_staff |
| visitor | 5% | visitor, mobility_impaired, conference_attendee, prospective_student_with_parent |

**生成參數摘要：**

```
Persona                              speed  comply  panic  obs_err  delay  famil
campus_security                       1.20    1.00   0.00     0.50      0   1.00  ← 最佳
student_athlete                       1.40    0.60   0.10     0.80      0   0.80
graduate_student                      1.20    0.90   0.10     0.80      0   0.80
student_with_anxiety                  0.80    0.60   0.85     2.20      3   0.40  ← 最高 panic
freshman_student                      1.20    0.40   0.80     2.50      3   0.20  ← 最低 compliance
mobility_impaired                     0.30    0.90   0.60     1.50      2   0.40  ← 最慢速度
visitor/prospective_*                 0.6-0.8  0.60-0.80  0.70-0.80  2.20-2.50  3  0.10  ← 不熟悉 shelter
```

### Persona 分配策略（`batch_runner.py`）

```
student → young_student (30%), freshman_student (20%), graduate_student (20%),
          international_student (10%), student_with_anxiety (8%), student_athlete (7%),
          part_time_student (5%)
faculty → senior_faculty (45%), junior_faculty (35%), adjunct_instructor (20%)
staff   → staff_admin (30%), facilities_staff (20%), healthcare_staff (15%),
          research_scientist (15%), campus_security (10%), it_staff (10%)
visitor → visitor (35%), conference_attendee (30%), prospective_student_with_parent (20%),
          mobility_impaired (15%)
```

### 修改的檔案

| 檔案 | 修改內容 |
|------|---------|
| `llm_behavior_profiler.py` | PERSONAS 從 5 擴充至 20 種（4 個 role 類別） |
| `agent_profiles.json` | 重新生成：20 persona 的定量行為參數 |
| `agents/base_agent.py` | 加入 persona、speed_multiplier 等 7 個欄位（預設值保持向後相容） |
| `agents/ped_agent.py` | step() 改為 `EVAC_SPEED_WALK * self.speed_multiplier` |
| `config.py` | 新增 `EVAC_ROLE_WEIGHTS` 字典，取代原本 faculty/staff 二元分割 |
| `batch_runner.py` | `_PERSONA_WEIGHTS` 更新為 4 個 role；`_build_agents()` 使用 role weights 隨機抽樣 |
