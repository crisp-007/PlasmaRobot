# L515 手眼稳定数据段求解记录（2026-07-29）

## 目的与边界

L515 重新安装后重新采集手眼数据。原始数据集共有 62 组，但固定板一致性检查表明
`img_059.png` 开始出现离散的空间关系变化。因此本次只使用确认未发生该变化的
`img_001.png` 至 `img_058.png` 求一个离线候选版本。

本次操作没有修改正式矩阵：

- 正式配置仍为 `config/camera_to_gripper.yaml`。
- 没有切换标定注册表。
- 没有开放机械臂运动许可。
- 原始 62 组数据没有删除或覆盖。

## 数据

原始数据集：

```text
/home/larusxu/Data/script/pointcloud_get/calibration/calib_data/handeye_l515_20260729_161944_precision_v1
```

稳定段快照：

```text
/home/larusxu/Data/script/pointcloud_get/calibration/calib_data/handeye_l515_20260729_161944_precision_v1_stable_001_058
```

快照包含 58 张图像和 58 行机械臂位姿，最后一组为 `img_058.png`。

`img_054.png` 至 `img_058.png` 相对前 53 组固定板参考位置的平移残差为
`1.98, 5.96, 7.09, 3.34, 3.81 mm`。从 `img_059.png` 开始，残差突然增大为
`36.58, 38.40, 47.94, 44.66 mm`，且主要集中在同一基座方向。因此
`img_059.png` 至 `img_062.png` 不参与本候选求解。

## 求解命令

```bash
source /home/larusxu/Data/activate_robot_arm.sh
cd /home/larusxu/Data/script/pointcloud_get/calibration
python3 handeye_calibrate_easy.py \
  --calibrate --method AUTO \
  --data_dir calib_data/handeye_l515_20260729_161944_precision_v1_stable_001_058 \
  --no_register
```

AUTO 在 PARK、TSAI 和 HORAUD 主算法中选择 TSAI。

## 候选矩阵

矩阵约定为 `gripper_T_camera`，平移单位为米：

```yaml
T_camera_to_gripper:
  - [-0.876102694, -0.482023923,  0.009849254, -0.036678083]
  - [ 0.482100079, -0.876081197,  0.007826272, -0.061625162]
  - [ 0.004856296,  0.011604944,  0.999920868,  0.029686178]
  - [ 0.0,          0.0,          0.0,          1.0]
```

相机原点在 Link6/夹爪坐标系中的平移为：

```text
[-36.678083, -61.625162, 29.686178] mm
```

候选相对当前旧矩阵变化为 `6.414586 mm / 3.656561 deg`。该旋转变化与相机重新安装
相符，旧矩阵不能视为本次安装状态的正式结果。

## 离线质量

| 指标 | 结果 |
|---|---:|
| 有效图像 | 58 / 58 |
| 固定板平移中位 / 最大 | 2.541 / 6.915 mm |
| 固定板旋转中位 / 最大 | 0.646 / 1.474 deg |
| 留一法平移最大影响 | 1.432 mm |
| 留一法旋转最大影响 | 0.125 deg |
| 五折平移中位 / 最大 | 2.675 / 6.999 mm |
| 五折旋转中位 / 最大 | 0.706 / 1.570 deg |
| TSAI/PARK/HORAUD 最大差异 | 0.368 mm / 0.352 deg |

平移、留一法和主算法一致性均通过。严格求解仍未通过：

- `fixed_board_rotation`：中位 `0.646 deg`，要求不大于 `0.500 deg`。
- `kfold_rotation`：中位 `0.706 deg`、最大 `1.570 deg`，要求分别不大于
  `0.500 deg` 和 `1.500 deg`。

候选文件：

- `records/camera_to_gripper_stable_001_058_candidate_20260729.yaml`
- `records/handeye_stable_001_058_quality_20260729.yaml`

## 使用结论

该矩阵是用户要求的第一版稳定段候选，可以用于离线比较和后续盲验证，但当前仍标记为
`quality_accepted: false`。在独立姿态固定板验证、深度/彩色平面复核和入口圆头盲测完成前，
不得把它注册为正式运动矩阵，也不得据此开放真实机械臂运动许可。

## 用户批准暂用更新

2026-07-29，用户明确决定使用该矩阵替换旧手眼矩阵。软件当前配置和标定注册表已切换到：

```text
provisional_tsai_20260729_stable_001_058
```

此次切换沿用项目已有的 `user_approved_provisional_use` 机制，允许点云、坐标转换、法向估计
和 RViz 预览。严格质量结果仍保留为未通过，真实接触和自动运动许可不会因本次切换自动开放。
