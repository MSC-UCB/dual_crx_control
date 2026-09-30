# Dual CRX admittance hand guiding：最小實作與驗證

日期：2026-09-29。單檔實作已完成，已以 bimanual mock 與合成 wrench 驗證；未連接或移動實機。

## 採用方案

新增一個 `scripts/admittance_hand_guiding.py`，內含 GUI 與左右獨立的 admittance。
操作座標與 FK／IK 終點統一採用各自的 `fanuc_flange`，直接重用現有 FK、IK 與
joint target helper。啟動既有 bimanual／dual_crx 控制鏈後，另外手動開 GUI，左右預設 Disabled。

```text
既有 wrench + joint_states
    → 以 fanuc_flange 做 FK，取得實際 pose
    → filter / deadband / admittance / limits，算出 T_cmd
    → 既有 IK，解出 joint targets
    → /crx5ia/joint_targets
    → 既有 interpolator → position controller → FANUC
```

**對原 prompt 的簡化：** Cartesian command 只存在於 script 內的 `T_cmd`，
實際 ROS 輸出是既有 `JointState`。現有專案沒有外部 Cartesian pose controller 或 TCP pose topic，
因此不增加 PoseStamped topics、pose adapter、橋接 node 或新的 controller。
這個方案保留 Cartesian admittance 的運動計算，但不提供原 prompt 所述的 ROS pose command 介面。

第一版也不加入 frame 切換、TF wrench transformation、tare／重力補償、資料錄製、曲線圖、
設定檔儲存、reset service、額外 command arbitration 或自動回初始位功能。

## 沿用的 topic 與工具

`{side}` 為 `left`／`right`：

| 用途 | Topic | 型別 |
| --- | --- | --- |
| Wrench | `/crx5ia/{side}/force_torque_sensor_broadcaster/wrench` | `geometry_msgs/msg/WrenchStamped` |
| 實際關節回授 | `/crx5ia/{side}/joint_states` | `sensor_msgs/msg/JointState` |
| 命令 | `/crx5ia/joint_targets`，包含啟用手臂的六軸／十二軸 | `sensor_msgs/msg/JointState` |
| 模型 | `/crx5ia/robot_description` | `std_msgs/msg/String`，transient local |
| 碰撞保護狀態 | `/crx5ia/collision_force_limiter/state` | `std_msgs/msg/String`，transient local |

重用 [CRXKinematics](../src/dual_crx_control/robot/kinematics.py)、
[DampedLeastSquaresIK／pose_error](../src/dual_crx_control/robot/ik_solver.py)、
[JointTargetClient](../src/dual_crx_control/interpolation/client.py) 與
[joint_config](../src/dual_crx_control/robot/joint_config.py)。
不繼承既有雙臂軌跡 node，避免帶入它的成對啟動、初始化軌跡與整體 abort 行為。

```python
model = CRXKinematics(robot_description, f'{side}_fanuc_flange')
solver = DampedLeastSquaresIK(model)
```

FK、Jacobian、IK 都會以這個終點計算，求出的仍是同一隻手的六個 joint positions。
目前 URDF 的 `tcp` 相對 flange 有 35 mm 偏移；改以 flange 旋轉時，該 TCP 會隨之移動。

目前 wrench label 正是 `left_fanuc_flange`／`right_fanuc_flange`。
啟動時記錄並檢查 label；空值、不符或運行中改變就拒絕該手 Enable／進入 Fault。
實機前仍須確認實際數值的座標軸、力矩原點與此 label 一致；tool compensation 不等同座標轉換。
原本「20 mm 施力點 → 0.2 Nm」的估計也要改以 flange 原點核對，不自動修改 wrench。

## 單檔程式結構

只需要三個 class：

- `ArmAdmittance`：單臂訂閱、資料時間、FK／IK、filter、v／T_cmd、Enable／Disable／Fault 與命令產生。
- `AdmittanceNode`：建立左右 instance、控制 timer、統整有效 targets、GUI 請求與狀態快照。
- `AdmittanceWindow`：Tkinter 左右面板，M／D 輸入、Apply、Enable／Disable、狀態顯示。

GUI main thread + 一個 ROS executor background thread。GUI 只透過 queue／短 lock 的快照交換資料，
元件只在 GUI thread 更新；Disable／關窗 event 優先於參數更新。
每臂自己的錯誤只清掉自己的 target，不用全域 failed flag。IK 採有限迭代並檢查耗時，
解算後若資料已過期就不發布。這是簡單的 soft real-time 架構，不承諾兩臂 CPU 排程隔離。

每個 timer tick 把成功手臂的 targets 合成一筆 `JointState`，選用對應單臂／雙臂
`JointTargetClient`，不在共用 depth-1 topic 連續送兩筆單臂命令。
沒有額外 pose topic 或非同步 IK worker，也就不需要 pose 回授橋接與 session 協定。

## 設定區與控制計算

所有初始值放在 script 頂端。GUI 只讓左右手各自調整六軸 M／D；K 固定為零。

| 設定 | 初值 |
| --- | --- |
| M，順序 `[x,y,z,rx,ry,rz]` | `[20,20,20,0.08,0.08,0.08]` |
| D，同上 | `[100,100,100,0.4,0.4,0.4]` |
| 分軸 deadband | `[1.5,1.5,1.5,0.03,0.03,0.03]` N／Nm |
| 一階 low-pass | 六軸各自 8 Hz |
| 分軸速度上限 | `[0.10,0.10,0.10,0.5,0.5,0.5]` m/s／rad/s |
| 合速度上限 | 平移 norm 0.10 m/s、旋轉 norm 0.5 rad/s，避免斜向速度增至 √3 倍 |
| 分軸加速度上限 | `[0.25,0.25,0.25,1.0,1.0,1.0]` m/s²／rad/s² |
| 控制／GUI 頻率 | 100 Hz／20 Hz，下游 `input_rate_hz` 對齊 100 |
| Enable force gain ramp | 0.5 秒 |
| M／D 平滑更新 | 0.3 秒 |
| Wrench／joint feedback timeout | 各 0.10 秒；實際 pose 年齡沿用其 joint feedback 年齡 |
| Command–actual 誤差上限 | 平移 0.02 m、最短旋轉角約 5° |
| 合法控制 dt | 0.002–0.05 秒 |

M 平移單位 kg、旋轉 kg·m²；D 平移 N·s/m、旋轉 Nm·s/rad。
先保留使用者指定的 M／D，不因操作點改動而自動重算參數。
不另外加入可選的 effective force／torque cap；速度／加速度限制與既有碰撞 limiter 已保留。

每隻手的流程：

1. 新 wrench 逐軸 LPF：`alpha=1-exp(-2*pi*fc*dt_sample)`。只對新樣本更新一次。
2. 逐軸連續 deadband：`w_eff=sign(w_filtered)*max(abs(w_filtered)-threshold,0)`。
3. `a=(gain*w_eff-D*v)/M`，限制加速度，以實測 dt 更新 v，再限制速度。
   同時確認最終速度變化未違反加速度限制，必要時縮短這次速度增量。
4. `T_candidate=T_cmd*Exp_SE3(v*dt)`，v 與 wrench 都使用當前選定的 flange/body 分量。
   以真正 SE(3) exponential 包含旋轉／平移耦合，使用 SciPy Rotation 與小型 left-Jacobian，
   不累加 Euler angles。FK／目標 pose 的 matrix 都表達在 world。
5. 檢查 candidate 與實際 pose 的誤差、IK 解與 joint limits，通過後才提交 T_cmd／joint target。
   拒絕時 fault 該手，不累積未被接受的 pose 位移。

wrench 跟隨實際 flange，右乘增量對應 command flange；第一版以小追蹤誤差近似兩者一致，
由誤差 watchdog 限制偏離，不處理大落差的 moving-frame 補償。
K=0；放手後 v 由 D 衰減至零並保持最後位置，不回啟用點。

用 monotonic 實測 dt。太小就略過並累積時間；非正、非有限或太大就 fault。
為容許即時調 M／D，Euler 積分可做有限 substeps：每步 `h*max(D/M) ≤ 0.25`、最多 20 步；
不能滿足就拒絕 gain 更新／fault，不能靠 M／D 為正便假定穩定。
GUI Apply 原子提交一整組有效 M／D，平滑趨近新值，錯誤輸入保留原參數。

加入 deadband 後，原始單軸 10 N 對應約 0.085 m/s，0.2 Nm 對應約 0.425 rad/s。
0.2 秒是未飽和模型的 M／D 時間常數；filter、ramp、加速度限制與底層延遲會改變實際反應。

## 保留的基本行為與保護

- 啟動左右皆 Disabled。Enable 前確認該手資料／frame／模型有效、命令 subscriber 存在、
  既有 collision limiter 已 ARMED；操作時只開一個運動命令來源。
- Enable 用當下實際 flange pose 初始化 command、v 清零、重設 filter 與時間，再開始 ramp。
  初始化成功前不發布命令；GUI 顯示 Disabled／Enabling／Enabled／Fault。
- Disable 優先在下一個控制 callback 封鎖該手發布、v 清零並清掉 pending target。
  再次 Enable 必須重新讀實際 pose，不能復用舊 command。
- 資料逾時、NaN／Inf、frame 改變、pose 追蹤誤差、IK／joint limits 或 dt 異常，都讓該手 Fault。
  另一手正常時仍可送出自己的 target。
- 既有 interpolator 可能完成最後接受的 segment 才保持位置，因此 Disable 不是急停。
  使用者先停止其他 motion scripts、確認沒有尚未完成的運動，再 Enable GUI。
- 既有 20 N collision limiter 保持運作；它超力／斷訊時仍會停雙臂，這是獨立控制的全域保護例外。
  收到 TRIPPED／STOP_FAILED 時 GUI 雙臂都停止發布。其 state 是事件 topic，不能因沒有新字串就當成斷訊。
- 關窗／Ctrl+C 先由 ROS thread disable 兩臂，再停止 timer／executor、關閉 ROS。
  GUI 用 `after()` 等待背景清理，不無限阻塞視窗。

GUI 顯示原需求中的 filtered／effective 六軸 wrench、虛擬線／角速度、資料年齡與錯誤訊息即可，
不加波形圖或額外診斷面板。Tkinter、NumPy、SciPy、PyKDL 與既有 helpers 已確認可 import。

## 修改與驗證範圍

程式只新增一個 script，加上 `CMakeLists.txt` 安裝入口、必要的 `python3-tk` 依賴與 README 用法。
測試另外放一個檔案，不新增 ROS package、不修改 FANUC driver、URDF 或其他 motion scripts。
既有 bimanual launch 不需再增加 admittance 參數。

必要測試集中在以下項目：

1. Syntax／import／安裝檢查，import 不開 GUI 或啟動 ROS。
2. 六軸正負 deadband、LPF、阻尼衰減、速度／加速度限制與 gains／dt 異常。
3. SE(3) 同時旋轉平移、小角度與非 identity 初始姿態；以 `scipy.linalg.expm` 交叉驗證。
4. Enable／Disable／re-enable、單側 timeout／IK fault 不阻擋另一側、關窗停止發布。
5. 隔離 ROS domain 的 mock joints 與 flange-frame wrench：驗證 FK → admittance → IK →
   joint targets，以及既有碰撞 limiter 的全域停止；有可用 display 再測 GUI 按鈕與即時參數。

先用既有預設插補模式測試，100 Hz 對齊目前預設 input rate；實測不足再調整，第一版不做插補模式比較工具。
既有 interpolator 的共同 horizon 可能影響雙臂時間行為，本次獨立的是各臂 admittance 狀態與 fault handling。

實機前核對 wrench 的實際 frame／力矩原點、flange 作為旋轉中心是否符合操作需求，
以及加速度／追蹤門檻。沒有確認前不啟動實體 hand-guiding motion。
實作檔為 [scripts/admittance_hand_guiding.py](../scripts/admittance_hand_guiding.py)，
測試為 [tools/test_admittance_hand_guiding.py](../tools/test_admittance_hand_guiding.py)。

## 執行與目前驗證

先用原本的 dual_arm 或 bimanual launch 啟動控制鏈；非 read-only 模式、collision limiter 保持開啟。
另一個終端 source ROS 與 workspace 後執行：

```bash
ros2 run dual_crx_control admittance_hand_guiding.py
```

沒有新增 launch 參數或修改 bimanual launch。GUI 關閉時只停止自己的命令來源；
啟用前先確認另一個 motion script 沒有殘留目標。另有輕量的 publisher 數量檢查，
偵測到其他 joint-target publisher 時拒絕啟用／fault；這不是新的命令仲裁層。

已完成 `colcon build --symlink-install --packages-select dual_crx_control`、`py_compile` 與文件連結檢查。
啟用 mock／GUI 選項的測試共 **41 項通過**。
已驗證：單檔語法／安裝、數值積分與保護、獨立 Enable／Disable、舊 command 的取消、
完整 bimanual mock 的雙臂位移與單側停用，以及 20 N 碰撞 limiter 的雙臂停止。
GUI 測試使用 Tk 視窗與背景 ROS thread，假回授持續更新，檢查按鈕、M／D 更新及關窗清理。
測試使用隔離 localhost ROS domains 184／185／186，未啟動 physical driver。

重跑（在 package 目錄、source ROS 與 workspace 後）：

```bash
ADMITTANCE_ROS_MOCK=1 ADMITTANCE_GUI_TEST=1 \
  ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST OPENBLAS_NUM_THREADS=1 \
  python3 -m pytest -q tools/test_admittance_hand_guiding.py
```

沒有 display 時不設定 `ADMITTANCE_GUI_TEST`；完整 mock 需要 workspace 已建置 bimanual package。
純 mock broadcaster 可能發布 NaN，正式 GUI 不會因此放寬有效資料條件；完整測試將 GUI 與 limiter
的 wrench 訂閱 remap 到測試專用 topic，注入正確 flange frame 的合成資料。

實際 force frame／力矩原點、手推感受、接觸環境穩定性、底層延遲與實際停車距離仍未在硬體驗證。
