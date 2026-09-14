# 鲁棒手眼离线重估记录（2026-07-27）

## 目的与边界

使用 2026-07-23 的 40 组 ChArUco 数据降低手眼矩阵对高影响样本的敏感性。本过程纯
离线，不启动相机、不连接机械臂、不修改正式手眼 YAML，也不向 GUI 加载候选。

## 新增工具

- `scripts/export_charuco_samples.py`：读取原始图片、彩色内参和 `robot_poses.csv`，导出
  同步的 `T_base_to_gripper` 与 `T_camera_to_board` PnP 缓存。
- `src/robust_handeye_analyzer.cpp`：C++ Huber/LM 联合 SE(3) 优化器，同时估计
  `T_camera_to_gripper` 和固定 `T_base_to_board`，比较逐步剔除与预定义高影响组合，并做
  五折留出评估。

复现命令：

```bash
python3 src/third_party/plasma_robot/tools/eye_hand/scripts/export_charuco_samples.py \
  --dataset /home/larusxu/Data/script/pointcloud_get/calibration/calib_data/handeye_l515_20260723_152410 \
  --output src/third_party/plasma_robot/tools/eye_hand/records/handeye_pnp_samples_20260723.yaml

install/plasma_eye_hand/lib/plasma_eye_hand/robust_handeye_analyzer \
  --dataset src/third_party/plasma_robot/tools/eye_hand/records/handeye_pnp_samples_20260723.yaml \
  --current-yaml src/third_party/plasma_robot/tools/eye_hand/config/camera_to_gripper.yaml \
  --output src/third_party/plasma_robot/tools/eye_hand/records/camera_to_gripper_robust_candidate_20260727.yaml \
  --max-exclusions 5 --folds 5
```

## 结果

在保留不少于 36 组的约束下，最优场景为 `robust_trim_4`：

| 指标 | 当前/原报告 | 鲁棒候选 |
|---|---:|---:|
| 固定板平移中位 | 3.035 mm | 1.836 mm |
| 固定板平移最大 | 7.231 mm | 4.870 mm |
| 五折平移中位 | 3.208 mm | 2.012 mm |
| 五折平移最大 | 7.317 mm | 5.172 mm |
| 固定板旋转中位 | 0.581 deg | 0.483 deg |
| 固定板旋转最大 | 1.635 deg | 1.193 deg |
| 五折旋转中位 | 0.580 deg | 0.522 deg |
| 五折旋转最大 | 1.734 deg | 1.254 deg |

剔除样本：`img_002.png`、`img_008.png`、`img_027.png`、`img_038.png`。

候选相对当前矩阵变化为 `8.736 mm / 0.138 deg`。主要收益在平移，但五折旋转中位仍未
通过 `0.500 deg` 门限。`camera_to_gripper_robust_candidate_20260727.yaml` 因此固定为：

```yaml
status: offline_candidate_only
validated: false
execution_allowed: false
motion_commands_published: false
```

## 结论与后续

鲁棒重估证明现有数据能够把标定板平移一致性改善到约 2 mm 中位水平，但尚不足以直接
替换正式矩阵。并且 L515 深度平面相对彩色 PnP 仍有 `-9.503 mm` 系统偏移；路径来自
深度坐标系，单独优化彩色 PnP 手眼不能消除该误差。

下一步先采集标定板在至少三个距离、每个距离多个画面位置的彩色角点与对齐深度平面，
离线拟合深度尺度/偏置和 `T_color_depth` 软件候选。随后使用未参与本次 40 组筛选的新姿态
验证鲁棒手眼；所有离线检查通过后，再进行圆头入口非接触盲测。
