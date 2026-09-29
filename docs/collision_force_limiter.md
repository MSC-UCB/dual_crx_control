# 雙臂碰撞力限制：最小改動方案評估

日期：2026-09-29。第一版已實作並通過軟體與 bimanual mock 整合測試，尚未連接實機驗證停止。
依據本機 `dual_crx_control`（`a44c9d4`）與 `fanuc_driver`（`a5a88ae`，含工作目錄修改）評估。

## 實作概述

已新增一個獨立的 Python `collision_force_limiter` node，同時監看左右手的 wrench。
任一手臂任一力分量的絕對值超過 20 N，就要求**雙臂停止運動並關閉 Stream Motion**，
觸發後鎖住，不因外力下降而自動恢復。沿用 ROS 2 controller manager 與 FANUC driver
現有停止介面，第一版不需要修改 Ruckig、其他插補器或各個 motion script。

這是碰撞力觸發停止功能，不調整速度上限。Python／DDS／service 路徑不具硬即時保證，
也不能取代 FANUC 控制器的碰撞保護或硬體急停；停止反應時間需實測。

需求中的「launch 預設打開」在此解讀為新增 `collision_force_limit:=true` 開關。
現有 `left_`、`right_` joint prefix 與 `/crx5ia` namespace 維持原本用途，不拿 `prefix` 當功能開關。

## 判定規則

```text
trip = max(abs(left.Fx), abs(left.Fy), abs(left.Fz),
           abs(right.Fx), abs(right.Fy), abs(right.Fz)) > 20.0 N
```

- 採各軸絕對值，正負方向都保護；不是只檢查正值，也不是計算三軸合力。
- 嚴格大於門檻才觸發，正好 20 N 不觸發。
- 只檢查 `wrench.force`；`wrench.torque` 單位是 Nm，不共用 20 N 門檻。
- 啟用監測後，一筆有效超標資料即觸發，第一版不加入平均、濾波或連續 N 筆確認造成額外延遲。
- 使用目前回傳的力值，不在啟動時自動扣除 offset／歸零，以免把原本存在的接觸力消掉。

現有 broadcaster 將 frame 標為 `left_fanuc_flange`／`right_fanuc_flange`，但 driver
只是把封包的力分量直接送出，沒有做座標轉換。實作驗收前要確認 FANUC 回傳值的實際座標系、
負載設定與重力補償；僅設定 `frame_id` 不能證明它就是該座標系的外力。
第一版的 XYZ 指現有感測回授分量，不另外轉到 world frame。

## 現有程式已提供什麼

| 項目 | 程式依據與意義 |
| --- | --- |
| 雙臂力回授 | [dual_arm.launch.py](../launch/dual_arm.launch.py) 已啟動左右 `force_torque_sensor_broadcaster`；[controller 設定](../config/dual_arm_controllers.yaml) 使用標準 `ForceTorqueSensorBroadcaster` |
| 輸入 topic | `/crx5ia/left/force_torque_sensor_broadcaster/wrench` 與 `/crx5ia/right/force_torque_sensor_broadcaster/wrench`，型別 `geometry_msgs/msg/WrenchStamped` |
| 力單位與來源 | `fanuc_driver/fanuc_libs/stream_motion/include/stream_motion/packets.hpp` 的 `force_x/y/z` 註明 N；CRX xacro 預設 `force_sensor_type=1`（embedded） |
| 既有命令行為 | [interpolation/node.py](../src/dual_crx_control/interpolation/node.py) 會完成／保持最後目標；只停 target publisher 不等於中斷 Stream Motion |
| 停止 position control | `FanucHardwareInterface::prepare_command_mode_switch()` 停用 position interface 時呼叫 `FanucClient::stopMotionControl()`，關閉 motion control、等待並 abort RMI |
| 關閉 Stream Motion | `FanucHardwareInterface::on_deactivate()` 呼叫 `FanucClient::stopRealtimeStream()`，停止 streaming thread、等待狀態、abort RMI 並送 stop packet |
| ROS 介面 | 本機 Jazzy 已有 `controller_manager_msgs/srv/SwitchController` 與 `SetHardwareComponentState`，不用另開 FANUC 連線 |
| Hardware 名稱 | 左右使用各自的 controller manager，但 xacro 的 hardware component 名稱都為 `crx5ia`；不是 `left_crx5ia` 或 `right_crx5ia` |

另外，Stream Motion protocol version <= 3 的相容分支會把力填成零。
`WrenchStamped` 沒有 `fs_type`／感測有效旗標，因此「有收到零值」不能證明有有效碰撞力回授。
首次實機部署須確認支援力回授的協定／感測器，而且施加已知方向的小力會改變讀值。
若需程式自動確認有效性，後續可增加 `fs_type`／driver status 的回授，不假設純 wrench 能判定這件事。

## 停止流程：重用 controller manager

第一版建議每側走以下流程，左右兩個流程並行啟動，不等左側完成才開始右側：

1. 收到超標資料後立即鎖住 `TRIPPED`，記錄來源手臂、軸、力值、門檻和 monotonic 時間。
2. 對兩側 `/crx5ia/{side}/controller_manager/switch_controller` 發出非同步請求：
   `deactivate_controllers=["forward_position_controller"]`、`activate_controllers=[]`。
   使用 `BEST_EFFORT` 與有限 timeout，另檢查回應；不能用無限等待。
3. 每側 controller 停用回應到達後，對該側
   `/crx5ia/{side}/controller_manager/set_hardware_component_state` 發出請求：
   `name="crx5ia"`、`target_state.id=2`（`PRIMARY_STATE_INACTIVE`）。
   驗證 `ok` 及回傳狀態確實是 inactive；必要時以 `list_hardware_components` 再確認。
4. 任一側第一步 service 失敗／逾時，仍嘗試該側 hardware deactivation，
   不把「controller 停用成功」當成唯一能繼續送停止要求的條件。
   另一側持續完成自己的停止流程。
5. 兩側回報 inactive 才記為「軟體停止流程完成」。無回應／失敗保留為 `STOP_FAILED`，
   清楚標出哪一側未確認，不把已送出請求或 topic 靜止當作已停止。

先停 controller 是為了走既有停止運動流程；再停 hardware 才明確要求結束 Stream Motion。
只做 `switch_controller` 可能保留狀態通訊，不能直接等同關閉整個 stream。
實作前應先以 mock 驗證本機 controller manager 的 lifecycle 行為；若證實可在 controller
active 時直接停用 hardware 並正確處理 controller，則可刪除第一個 service 階段以減少延遲。
目前尚未驗證這個簡化，不直接當作已成立。

用 `rclpy` 的 `call_async()` 與 timer 管理兩側進度及 deadline，callback 裡不要
`sleep()`、同步等 service 或執行 shell。每側同一階段只保留一筆 outstanding request；
逾時不代表 server 已取消執行，不能每筆 wrench 都再送一輪停止請求。

driver 的 `stopMotionControl()` 與 `stopRealtimeStream()` 含約一秒的等待迴圈，還有
thread join、網路接收及 RMI 呼叫；這不是整個停止程序的一秒上限。
`stopRealtimeStream()` 的接收失敗也可能在送出最後 stop packet 前拋例外。
因此第一版須量測「收到超標 → 發出要求 → hardware inactive → 實際速度降至零」，
不能只拿 500 Hz control loop 推算成 2 ms 停車。

建議 launch 加上 limiter 的 `OnProcessExit` → `Shutdown`，關閉自動 respawn，
讓 limiter 意外退出時不會默默留下沒有監測的控制系統。
停止流程失敗並超過有限的總等待時間時，也可退出 limiter 觸發整個 launch 的關閉。
這是盡力清理的備援；driver 的 `on_shutdown()` 本身沒有執行停止，清理還依賴析構／程序退出，
不能把 launch shutdown 或強制 kill 視為已確認硬體停止。
這個最小方案也不偵測 limiter 程序卡死；若需要涵蓋此故障，需再加獨立 watchdog 或 driver 層保護。

## 初始化忽略與觸發後恢復

建議使用 `WAITING → WARMUP → ARMED → TRIPPED` 的單向狀態流程。

- `WAITING`：等待兩側 controller／hardware 準備好、停止服務可用，且雙側收到有限數值的
  新 wrench。超時就報錯並結束 bringup，不永久留在「初始化忽略」。
- `WARMUP`：從上述條件首次滿足起計時，例如 3 秒，暫不因超過 20 N 而觸發。
  持續檢查兩側資料是否新鮮；資料中斷時取消這次暖機，回到等待，但不延長總啟動 deadline。
- `ARMED`：暖機完成就檢查當下的新資料；若力一直高於門檻，立即觸發。
  不要求「力先降到 20 N 以下才啟用」，避免一直超標就一直忽略。
- `TRIPPED`：只進入一次停止流程。力降低、收到新 target 或重新連線都不自動恢復。
  第一版不提供 reset／自動 re-activate；確認接觸已解除、停止外部命令來源後，重新 launch。

一旦進入 `ARMED`，雙側任何一路資料逾時或出現 NaN／Inf，建議同樣觸發停止，
並把原因標成回授異常，不能再回到暖機來略過問題。
用 monotonic 接收時間判斷 freshness，訂閱採 sensor-data 相容 QoS、短 queue，避免處理大量舊資料。
這只能偵測 topic 層的中斷；若上游持續重發凍結的數值，仍需 driver 狀態／封包序號才能辨識。

此處「初始化」指 driver／感測資料啟動，不包含 motion script 的回初始姿態動作。
目前 canonical launch 不主動移動到初始姿態，也沒有全域的「所有初始化動作已完成」訊號。
若需求是連回初始姿態的移動都忽略，就需要 motion script 明確發出完成訊號，不能用固定 3 秒猜測。

最小版本的等待／暖機期間，limiter 尚未提供碰撞保護，既有命令入口仍能收到 target。
應明確印出並發布 `WAITING/WARMUP/ARMED/TRIPPED/STOP_FAILED` 狀態，只有 `ARMED` 後才開始外部 motion。
若要由程式保證暖機前完全不能開始外部運動，需增加啟動 gate，延後啟動 forward controller／interpolator；
這比單一監測 node 多一層 launch 協調，列為下一步，不假裝第一版已具備。

## Launch 參數建議

目前 launch 只對外提供啟用開關與力門檻兩個碰撞力限制參數。

| 參數 | 建議預設 | 用途 |
| --- | --- | --- |
| `collision_force_limit` | `true` | 預設啟用碰撞力限制 |
| `collision_force_threshold_n` | `20.0` | 任一軸絕對值門檻，須為有限正數 |

初始化與逾時設定集中寫成 node 內部常數，不提供 launch argument 或 ROS parameter：
雙側準備好後暖機忽略 3 秒、從 node 啟動至完成暖機最多 180 秒、啟用後任一側
超過 0.2 秒沒有新 wrench 就觸發停止。這些時間是初始建議，實機驗證後再調整常數。

stop service 的 deadline 可先固定在 node 內，例如每階段 3 秒、整個停止流程 8 秒，
實作時讓異常路徑遵守總 deadline。這些數值是錯誤回報的等待預算，不是允許機械臂繼續移動的時間。

`dual_arm.launch.py` 為主要接入點：

- `read_only:=false`：依 `collision_force_limit` 啟動 limiter。
- `read_only:=true`：不啟動停止用 limiter，因為不擁有運動控制權；明確記錄此模式無本功能。
- `mock:=true`：保留 limiter 狀態機供測試，以合成 wrench 驗證；mock 的零值／NaN 不代表實機感測可用。
  測試注入時將 limiter 的輸入 remap 到專用 topic，避免與 mock broadcaster 同時發布互相掩蓋。
- 舊的 `dual_arm_readonly.launch.py` 不是主要運動入口，第一版無需接入。

## 從 bimanual_manipulation 一起啟動

可以沿用同一個 limiter。已確認 `bimanual_manipulation/launch/bimanual_system.launch.py`
透過 `IncludeLaunchDescription` 引入 `dual_crx_control/launch/dual_arm.launch.py`，
所以 limiter 接在後者且預設啟用後，從 bimanual 啟動也會一起啟動，不需要再建立第二個 limiter。
目前 bimanual 對 CRX group 只 remap TF，沒有更動上述 wrench topic 或 controller manager 路徑。

已將 `collision_force_limit` 與
`collision_force_threshold_n` 加入 `bimanual_system.launch.py` 的 `ARM_OVERRIDES`。
既有程式會自動宣告這兩個參數、傳遞非空值；留空時沿用 dual_crx 的 `true`／`20.0` 預設，
不需要在兩個 package 重複維護預設值。

目前用法：

```bash
ros2 launch bimanual_manipulation bimanual_system.launch.py \
  collision_force_limit:=true collision_force_threshold_n:=20.0
```

全域 `read_only:=true` 仍會傳到 dual_crx，因此此時不啟動停止用 limiter。
正常碰撞觸發只要求 CRX 雙臂停止，limiter 留在鎖住狀態，不會主動命令 Sharpa 手指停止。
若 limiter 意外退出或停止失敗後觸發備援 `Shutdown`，此事件會關閉整個 bimanual launch，
包括 Sharpa、viewer 等程序；`GroupAction` 不會把 launch shutdown 限制在 CRX group。
整合測試需涵蓋這個差異，不能把正常雙臂停止與整機程序關閉混為一談。

## Minimum effort 的修改範圍

| 檔案 | 修改 |
| --- | --- |
| `src/dual_crx_control/collision_force_limiter.py`（新增） | node 與可獨立測試的門檻／停止狀態機 |
| `scripts/collision_force_limiter.py`（新增） | 安裝用的薄入口，呼叫上述 module |
| `launch/dual_arm.launch.py` | 僅新增啟用開關與力門檻兩個參數、條件啟動 node、limiter exit 時 shutdown 的 event handler |
| `bimanual_manipulation/launch/bimanual_system.launch.py`（另一個 repo） | 將上述兩個參數加入 `ARM_OVERRIDES`，由現有 include 路徑啟動同一個 limiter |
| `CMakeLists.txt` | 將新 script 安裝至 `lib/dual_crx_control`，加入必要測試 |
| `package.xml` | 補上直接使用的 `geometry_msgs`、`lifecycle_msgs` 依賴；目前已有 `rclpy`、`controller_manager_msgs` |
| `tools/test_collision_force_limiter.py`（新增） | 判定與狀態機測試、假的停止服務測試 |
| `tools/test_collision_force_launch.py`（新增） | 完整 bimanual mock launch 測試，需設定 `COLLISION_ROS_MOCK=1` |
| `README.md` | 啟用方式、初始化忽略範圍、觸發後需要重啟的行為 |

使用一個 module 加上薄 script 入口，讓測試可直接 import；沒有新增 ROS package、自訂 message、命令轉送層或修改 motion scripts。

| 方案 | 評估 |
| --- | --- |
| 獨立 node + 既有停止 services | **建議第一版**：改動集中於 `dual_crx_control`，容易觀察與測試；需驗證停止延遲 |
| 只停 target／interpolation publisher | 改動雖少，但既有目標、buffer 與 Stream Motion 可能繼續，無法滿足要求 |
| 觸發後只關整個 launch | 程式更少，但停止確認與錯誤處理較弱；適合作為備援清理 |
| 在 FANUC driver 讀取／stream thread 裡判定 | 可縮短反應路徑，但涉及 driver 修改、同步及雙臂聯動；若 service 路徑延遲不符需求再做 |

## 實作驗證順序

1. 軟體測試：左右六個分量各自的正負超標、20 N 邊界、合力大但每軸小於 20 N、
   NaN／Inf、暖機前後、暖機期間資料中斷、啟動逾時、監測後斷訊、觸發鎖住不自動恢復。
2. Fake service 整合：確認單側超標會對兩側發要求；一側延遲／失敗不阻塞另一側；
   controller 停用失敗仍嘗試停 hardware；逾時不重複灌入請求；回應不符不得記為成功。
3. 隔離 ROS domain 的 mock launch：檢查預設開關、read-only 分支、controller → hardware
   狀態轉換及 limiter 意外退出時 launch 行為。mock 不能驗證實際 stop packet 或停車距離。
4. 實機先靜止檢查：確認力回授有效、座標與負載意義，透過可控的力輸入觸發，確認兩側
   stream 都關閉；檢查關閉後是否仍有有效 wrench，而不是誤讀舊數值。
5. 最後做受控低速驗證：記錄偵測到停止的延遲、實際停止距離、兩臂停止時間差，並確認
   外部 publisher 持續送命令時不會恢復動作。若反應時間不合需求，升級到 driver／控制器層。

## 已完成驗證

- `colcon build --symlink-install --packages-select dual_crx_control bimanual_manipulation` 成功。
- `tools/test_collision_force_limiter.py`：34 項通過，涵蓋六個分量正負門檻、20 N 邊界、無效資料、初始化／斷訊、停止服務失敗及雙側 ROS service 呼叫；另有 6 項既有 bimanual 整合回歸測試通過。
- `COLLISION_ROS_MOCK=1 python3 -m pytest -q tools/test_collision_force_launch.py`：5 項通過，包含 bimanual 預設 20 N、覆寫 25 N、停用、read-only，以及 limiter 退出時關閉整個 launch。
- 完整 mock 測試確認正常觸發後兩側 hardware 都為 inactive、forward controller 都為 inactive，且外力下降不自動恢復。
- ROS 測試使用 localhost 與隔離 domain 183／184，所有 driver 都是 mock／fake，沒有連接、enable 或移動實機。實際 stop packet、停止距離與延遲仍待實機驗證。

在 package 目錄 source ROS 與 workspace 後可重跑：

```bash
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 -m pytest -q tools/test_collision_force_limiter.py
COLLISION_ROS_MOCK=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 -m pytest -q tools/test_collision_force_launch.py
```

狀態發布在 `/crx5ia/collision_force_limiter/state`（`std_msgs/msg/String`，reliable／transient local）。
正常停止後維持 `TRIPPED`；`STOP_FAILED` 會記錄未確認停止的一側並以非零退出，觸發 launch shutdown。
第一版會先停 controller 再停 hardware；尚未採用直接 deactivate active hardware 的簡化。
