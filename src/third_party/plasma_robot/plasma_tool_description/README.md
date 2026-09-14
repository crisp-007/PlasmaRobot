# Plasma tool description

该 ROS 2 描述包保存 ECO65 等离子末端的正式模型资源，不修改 RealMan 原厂
`rm_description` 包。

## 装配结构

```text
Link6
└── plasma_tool_base_link
    └── plasma_adapter_link
        └── plasma_spray_rod_link
            ├── Arm_Tip                  # 物理圆头兼容帧，318 mm
            └── plasma_motion_tcp        # 真实侧喷口中心，实际跟踪点
                ├── plasma_nozzle_tip    # 圆头末端，仅用于底部间隙计算
                └── plasma_spray_outlet  # 侧喷口别名，与运动 TCP 重合
```

装配固定关节使用 CAD 的局部 `+Y` 方向；整套 CAD 组件在 Link6 安装层使用
`Rx(+90 deg)`，将局部 `+Y` 映射到 `Link6 +Z`。过程坐标系
`plasma_motion_tcp` 的 `+X` 沿喷杆指向底部，`+Z` 指向侧面喷涂方向；每层
正负 180 度喷涂旋转绕 `+X` 进行。

当前试验默认组合：

```text
adapter_type: 3_4（内部模型规格，不在操作软件中显示）
spray_rod_type: 200_1x10
Link6 到运动 TCP: 轴向 0.310 m，侧向 0.002 m
Link6 到圆头末端: 轴向 0.318 m
运动 TCP 到圆头末端: [+X 0.008, 0, -Z 0.002] m
运动 TCP 到侧喷口中心: [0, 0, 0] m
```

三张现行图纸确认喷杆物理总长为 `210 mm`，其中 `10 mm` 插入枪体，枪口外露
仍为 `200 mm`。2026-07-30 现场重新确认实际装配轴向栈为端板 `8 mm`、固定罩
`70 mm`、喷头本体 `40 mm`、外露喷管 `200 mm`，所以 Link6 到圆头总长为
`318 mm`。圆头向后 `3 mm` 只到 `1x10` 喷口的近侧边；喷口轴向长度为
`10 mm`，所以喷口中心在圆头后方 `3 + 5 = 8 mm`，Link6 到运动 TCP 的
轴向距离为 `310 mm`。`118 mm` 是喷管开始处/喷头本体末端，不是过程运动 TCP。
`3_4` 喷杆外半径为 `2 mm`，因此运动 TCP 同时包含 `2 mm` 的侧向
表面偏移。径向偏移对远距离喷涂的距离影响很小，但保留它可保证正负 180 度
旋转以真实喷口中心为固定点。

现场安装横向偏差和完整姿态尚未完成坐标验证，因此
`config/tcp_3_4_200_1x10.yaml` 保持 `validated: false`，不得据此开放实机
自动运动。

## 2026-07-23 Arm_Tip 方向点动记录

在真实 `ECO65-BI` 三代控制器上确认当前控制器工具系为 `Arm_Tip` 后，使用
工具坐标系、5% 速度执行了 `+Z 2.0 mm` 和 `-Z 2.0 mm` 往返点动。两条
MoveL 指令均返回成功：

```text
点动前 TCP: (-251.035, -74.922, 501.518) mm
+Z 后 TCP:  (-252.334, -74.715, 500.032) mm
实际位移:   (-1.299, +0.207, -1.486) mm，长度约 1.98 mm
返回后 TCP: (-251.007, -74.930, 501.534) mm
返回位置误差: 约 0.033 mm
```

点动和返回闭环已通过；操作者已目视确认 `Arm_Tip +Z` 沿喷杆轴线朝喷嘴
尖端方向运动。该结果只完成了工具轴方向验收，尚未验证 Link6 安装原点和完整
六自由度 TCP，因此仍不得将 TCP 标记为 `validated`，也不得开放自动执行。

同日通过新增的只读详细查询接口读取控制器当前 `Arm_Tip`，其平移和欧拉角均
为零。使用同一组实时关节角比较控制器 UDP 零工具位姿和 ROS
`baselink -> Link6`，两者位置一致、四元数只差整体符号，因此控制器当前
`Arm_Tip` 实际是法兰零工具，并未保存喷嘴长度。软件必须继续使用
`T_gripper_to_tcp` 将运动 TCP 目标反算为法兰目标，不得把控制器名称正确误判为
TCP 参数已经写入。

2026-07-24 使用的 `302/310 mm` 软件值漏掉了 Link6 端板的 `8 mm`，已经由
`TOOL_AXIAL_REMEASUREMENT_20260730.md` 覆盖。按旧值生成的轨迹必须作废。

`3_4` 与 `6_8` 只选择不同直径的转接件和喷杆网格，同一长度/喷嘴规格
使用相同的轴向位置，但侧喷口表面半径不同。操作软件当前使用现场 `3_4`
模型。GUI 读取 YAML 的 `measurement` 字段，坐标转换节点读取同一文件中的
刚体变换；调整实测值时必须同步更新测量字段和矩阵，不能只改显示长度。

## 型号选择

全部可选组合记录在 `config/tool_variants.yaml`。目前包含：

- 转接件：`3_4`、`6_8`
- 喷杆长度：`150`、`200` mm
- 喷杆型号：`1x5`、`1x10`、`1x20`

默认模型：

```bash
ros2 launch plasma_tool_description display_tool.launch.py
```

选择其他组合：

```bash
ros2 launch plasma_tool_description display_tool.launch.py \
  adapter_type:=3_4 spray_rod_type:=150_1x20
```

launch 会验证组合是否存在，再将选择的网格、惯量和 TCP 局部位置交给
Xacro；无效型号会直接拒绝启动。

## 文件职责

```text
config/tool_variants.yaml             # 型号、惯量和局部 TCP 数据
config/tcp_3_4_200_1x10.yaml          # 侧喷口 TCP、圆头末端统一配置
reference/drawings/                    # 当前末端 DWG 原图和尺寸依据
reference/cad/                         # 未安装备选件的 CAD 归档
VERIFICATION_20260723.md               # 历史验证记录（顶部已标记作废项）
TCP_SPRAY_OUTLET_20260724.md           # 本次喷口 TCP 修正规则与验收步骤
urdf/plasma_tool_macro.xacro           # 可复用末端装配宏
urdf/rm_eco65_with_plasma_tool.urdf.xacro
launch/display_tool.launch.py          # 型号选择和 RViz 显示入口
meshes/                                # 底座、转接件和喷杆 STL
```

加固板方案已于 2026-07-29 撤回，当前 URDF 不加载加固板网格，
也不包含其 `2 mm` 轴向偏移。

## 后续实测

更换组合时必须为该组合生成独立 TCP YAML。正式执行前至少核对：

1. `Link6` 与公共底座的安装原点、方向和螺孔定位。
2. 实际转接件和喷杆型号。
3. `Link6` 到真实侧喷口 TCP 和圆头末端的位置。
4. `Arm_Tip +Z` 方向已通过实机往返点动和操作者目视确认；更换末端后需重验。
5. 已知点验证通过后才将 TCP 标记为 validated。
