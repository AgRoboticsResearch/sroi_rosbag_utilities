# schemes_compare — 遮罩方案对比 (raw / maskhalf / maskgripper)

对比三种 ORB-SLAM3 前处理遮罩方案对轨迹还原的影响，并给出生产方案。

## 结论（287 集，4 个 Vive + 2 个无 Vive session 验证）

| 方案 | 说明 | 结果 |
|---|---|---|
| `raw` | 不遮 | 大量坍缩（轨迹缩成真实运动零头），**不可用** |
| `maskhalf` | 底部 38/97 涂黑 (cutoff=292) | 修好坍缩，但偶有跟丢 |
| `maskgripper` | **只遮夹爪梯形**（左右红外各一套顶点 + 底部 13 行） | **质量等价 + 跟踪更稳** |

**数据**（以 maskhalf 为基准，纯方法间对比，不依赖 Vive）：
- 质量：两边都跟全的 278 集，maskgripper vs maskhalf 偏差中位 **0.5mm**（亚毫米，等价）。
- 稳健性：maskhalf 跟丢 9 集，maskgripper 跟丢 5 集；**maskgripper 救回 4 集** maskhalf 跟丢的。
- → **生产默认用 `maskgripper`**（保留更多特征 → ORB 不易丢跟踪；额外成本仅涂黑步骤略复杂）。

根因：画面底部的夹爪 + AprilTag 是高对比、会动的"毒特征"，破坏 ORB-SLAM 静态场景假设。基线/标定没问题（D405 18mm 基线本就对）。

## 生产用法（处理新数据）

跑 ORB-SLAM 前，对每个 episode 的 left_/right_ 涂黑夹爪梯形：
```bash
python3 schemes_compare/mask_gripper_trapezoid.py <schemes或episode目录>
```
梯形顶点（夹爪与相机刚性连接，画面里固定，只有手指开合；已跨两台设备验证通用）：
- 左眼：[(236,292),(417,292),(525,467),(127,467)]（顶左→顶右→底右→底左）+ rows≥467 全宽黑
- 右眼：[(182,292),(365,292),(444,467),(46,467)] + rows≥467 全宽黑
- 左眼夹爪居中→对称；右眼因立体视差左移 ~48px，故左右眼顶点不同。

## 完整对比流程（复现 mg≥mh）

```bash
# 1) 一个 session 全量做成 schemes(raw/maskhalf/maskgripper 各跑 ORB)
MASK_DATA=/path/to/data bash schemes_compare/build_session_schemes.sh 20260709_XXXXXX
# 2) 坐标变换(投影回RGB必需)
python3 transform_trajectory.py <session>-schemes --recursive
# 3) 三面板对比视频(每episode raw|maskhalf|maskgripper 并排)
python3 schemes_compare/viz_scheme_compare.py <session>-schemes
# 4) 完整度+偏差(以maskhalf为基准)
python3 schemes_compare/deviation_vs_baseline.py <session>-schemes
# 5) 跨session总表
python3 schemes_compare/aggregate_schemes.py
```
输出在 `<session>-schemes/_compare/`（视频 + deviation.csv）。

## 文件说明
- `mask_gripper_trapezoid.py` — **生产掩膜**（mg）。`mask_session.py` — maskhalf（cutoff=292，对比用）。
- `schemes_init.py` / `build_session_schemes.sh` / `run_orb_scheme.py` — schemes 结构搭建 + 批量 ORB。
- `viz_scheme_compare.py` — 三面板对比视频（复用 `visualization/visualize_traj_video.py` 的投影）。
- `deviation_vs_baseline.py` / `aggregate_schemes.py` — 完整度(输出帧/输入帧，丢帧=失败) + 偏差分析。
- `orb_vs_vive.py` — ORB vs Vive 动捕真值 Sim3 对齐（Vive 准的时候用）。

## 注意
- `T_CAM_EE`（相机→夹爪指尖变换，在 `visualization/visualize_traj_video.py`）是按本装置实测重标定的（指尖 +0.145m 前/−0.030m 下、pitch −30°）。换装置请核验绿点是否落夹爪中心。
- 脚本里的数据路径是示例（默认 `/home/ss/data/1000_onesb_labpicking`），用 `MASK_DATA` 环境变量或改 `BASE` 适配你的机器。
- Vive 动捕在遮挡时会出错，不能盲信当真值；用前需识别并剔除被遮挡 episode。
