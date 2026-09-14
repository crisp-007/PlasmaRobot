function slice_result = cavity_slicing_planes_matlab()
% =========================================================
% 残腔切片平面可视化（修正版）
%
% 修正内容：
% 1) 参考轴线显示段与切片平面放置逻辑统一：
%    - a 方向统一为指向残腔内部（深度方向）
%    - 开口端：参考轴线显示到包围盒端面（可加 SAFE_MARGIN_TOP）
%    - 底部端：参考轴线显示到距底部 SAFE_MARGIN_BOTTOM 处
% 2) 切片平面表示"喷嘴中间平面"
%    - 第一个切片平面位于底部有效端向开口方向偏移半个喷嘴长度
%    - 后续按固定间距向开口方向排列
%    - 当超出开口端有效范围时停止
% 3) 可视化：
%    - 残腔表面
%    - 参考轴线（修正后的有效显示段）
%    - 定向包围盒
%    - 所有切片平面
%    - 第一个切片平面（红色高亮）
%
% 显示设置：
% - 中文：黑体
% - 英文/数字：Times New Roman
% - 不加粗
% - 所有文字与坐标轴颜色统一为纯黑
% - 关闭 3D 外框 Box，只保留坐标轴方向线
%
% 依赖：
% - cavity_oriented_bounding_box_matlab.m
% =========================================================

%% -------------------------
% 用户参数区
%% -------------------------
SLICE_SPACING = 5;              % 切片间距（喷嘴轴向长度；若单位为m，则0.005=5mm）
SAFE_MARGIN_BOTTOM = 5;         % 靠近残腔底部的预留安全距离
SAFE_MARGIN_TOP = 0.0;          % 靠近开口端的预留距离
PLANE_SCALE_RATIO = 1.08;       % 切片平面尺寸相对于包围盒横向范围的放大倍数

SURFACE_COLOR = [0.82 0.82 0.82];
BBOX_COLOR = [0.10 0.70 0.20];
SLICE_PLANE_COLOR = [0.25 0.60 1.00];
FIRST_SLICE_PLANE_COLOR = [1.00 0.00 0.00];
STRAIGHT_AXIS_COLOR = [0.85 0.0 0.85];

%% -------------------------
% 字体与显示宏定义
%% -------------------------
FONT_SIZE_NORMAL = 15;              % 标准字体大小
CN_FONT = '黑体';                   % 中文字体：黑体
EN_FONT = 'Times New Roman';        % 英文/数字字体
FONT_WEIGHT_ALL = 'normal';         % 不加粗
TEXT_COLOR = [0 0 0];               % 纯黑色
AX_LINEWIDTH = 1.2;                 % 坐标轴线宽

%% -------------------------
% 1) 调用包围盒结果
%% -------------------------
fprintf('============================================================\n');
fprintf('残腔切片平面可视化（修正版）\n');
fprintf('首先调用 cavity_oriented_bounding_box_matlab() ...\n');
fprintf('============================================================\n');

fig_before = findall(0, 'Type', 'figure');
bbox_result = cavity_oriented_bounding_box_matlab();

% 关闭前一步弹出的窗口，避免界面重复
fig_after = findall(0, 'Type', 'figure');
new_figs = setdiff(fig_after, fig_before);
for i = 1:numel(new_figs)
    if isvalid(new_figs(i))
        close(new_figs(i));
    end
end

%% -------------------------
% 2) 读取局部坐标系与包围盒范围
%% -------------------------
u = bbox_result.u_axis;
v = bbox_result.v_axis;
a = bbox_result.a_axis;
o = bbox_result.origin_o;

u_min = bbox_result.u_min;
u_max = bbox_result.u_max;
v_min = bbox_result.v_min;
v_max = bbox_result.v_max;
a_min = bbox_result.a_min;
a_max = bbox_result.a_max;

% 统一 a 轴方向：要求 a 指向残腔内部，与深度方向同向
depth_axis = bbox_result.depth_axis(:).';
if dot(a, depth_axis) < 0
    a = -a;
    u = -u;   % 同时翻转一个横向基，保持右手系

    % 用新的局部坐标系重新计算顶点局部坐标与范围
    local_uvt = world_to_local(bbox_result.V, o, u, v, a);
    u_min = min(local_uvt(:,1)); 
    u_max = max(local_uvt(:,1));
    v_min = min(local_uvt(:,2)); 
    v_max = max(local_uvt(:,2));
    a_min = min(local_uvt(:,3)); 
    a_max = max(local_uvt(:,3));
else
    local_uvt = bbox_result.local_coords;
end

%% -------------------------
% 3) 定义有效轴向区间
% 约定：
% - a 方向指向残腔内部
% - a_min -> 靠近开口端
% - a_max -> 靠近残腔底部端
%% -------------------------
a_opening_limit = a_min + SAFE_MARGIN_TOP;        % 开口端有效下限
a_bottom_limit  = a_max - SAFE_MARGIN_BOTTOM;     % 底部端有效上限

if a_bottom_limit <= a_opening_limit
    error('有效切片区间无效，请检查 SAFE_MARGIN_TOP / SAFE_MARGIN_BOTTOM 参数。');
end

if isempty(SLICE_SPACING) || SLICE_SPACING <= 0
    error('SLICE_SPACING 必须为正数。');
end

%% -------------------------
% 4) 参考轴线显示段
%% -------------------------
axis_a_start = a_opening_limit;
axis_a_end   = a_bottom_limit;

axis_local_pts = [
    0, 0, axis_a_start;
    0, 0, axis_a_end
];

axis_world_pts = local_to_world(axis_local_pts, o, u, v, a);
axis_display_p0 = axis_world_pts(1, :);
axis_display_p1 = axis_world_pts(2, :);

%% -------------------------
% 5) 切片平面位置（喷嘴中间平面）
%% -------------------------
first_slice = a_bottom_limit - 0.5 * SLICE_SPACING;

if first_slice < a_opening_limit
    error('首个切片平面已超出有效范围。请减小 SAFE_MARGIN_BOTTOM 或减小 SLICE_SPACING。');
end

slice_positions = [];
ak = first_slice;

while ak >= a_opening_limit - 1e-12
    slice_positions(end+1) = ak; %#ok<AGROW>
    ak = ak - SLICE_SPACING;
end

num_slices = numel(slice_positions);

fprintf('开口端有效下限 a_opening_limit = %.6f\n', a_opening_limit);
fprintf('底部端有效上限 a_bottom_limit  = %.6f\n', a_bottom_limit);
fprintf('切片间距 SLICE_SPACING         = %.6f\n', SLICE_SPACING);
fprintf('首个切片平面位置 first_slice   = %.6f\n', first_slice);
fprintf('切片平面数量                   = %d\n', num_slices);

%% -------------------------
% 6) 构造每个切片平面的四个角点
%% -------------------------
u_center = 0.5 * (u_min + u_max);
v_center = 0.5 * (v_min + v_max);

u_half = 0.5 * (u_max - u_min) * PLANE_SCALE_RATIO;
v_half = 0.5 * (v_max - v_min) * PLANE_SCALE_RATIO;

plane_local_template = [
    u_center - u_half, v_center - v_half;
    u_center + u_half, v_center - v_half;
    u_center + u_half, v_center + v_half;
    u_center - u_half, v_center + v_half
];

slice_planes_world = cell(num_slices, 1);

for k = 1:num_slices
    ak = slice_positions(k);

    local_pts = [
        plane_local_template(1,1), plane_local_template(1,2), ak;
        plane_local_template(2,1), plane_local_template(2,2), ak;
        plane_local_template(3,1), plane_local_template(3,2), ak;
        plane_local_template(4,1), plane_local_template(4,2), ak
    ];

    world_pts = local_to_world(local_pts, o, u, v, a);
    slice_planes_world{k} = world_pts;
end

%% -------------------------
% 7) 按修正后的局部坐标系重建包围盒线框
%% -------------------------
bbox_local_corners = [
    u_min, v_min, a_min;
    u_max, v_min, a_min;
    u_max, v_max, a_min;
    u_min, v_max, a_min;
    u_min, v_min, a_max;
    u_max, v_min, a_max;
    u_max, v_max, a_max;
    u_min, v_max, a_max
];

bbox_world_corners = local_to_world(bbox_local_corners, o, u, v, a);

bbox_edges = [
    1 2; 2 3; 3 4; 4 1;
    5 6; 6 7; 7 8; 8 5;
    1 5; 2 6; 3 7; 4 8
];

%% -------------------------
% 8) 结果打包
%% -------------------------
slice_result = bbox_result;
slice_result.u_axis = u;
slice_result.v_axis = v;
slice_result.a_axis = a;
slice_result.local_coords = local_uvt;
slice_result.u_min = u_min;
slice_result.u_max = u_max;
slice_result.v_min = v_min;
slice_result.v_max = v_max;
slice_result.a_min = a_min;
slice_result.a_max = a_max;

slice_result.slice_positions = slice_positions;
slice_result.slice_planes_world = slice_planes_world;
slice_result.slice_spacing = SLICE_SPACING;
slice_result.a_opening_limit = a_opening_limit;
slice_result.a_bottom_limit = a_bottom_limit;
slice_result.axis_display_p0 = axis_display_p0;
slice_result.axis_display_p1 = axis_display_p1;
slice_result.bbox_world_corners = bbox_world_corners;
slice_result.bbox_edges = bbox_edges;
slice_result.first_slice = first_slice;

%% -------------------------
% 9) 可视化
%% -------------------------
fig = figure('Color', 'w', ...
             'Name', '残腔切片平面可视化（修正版）', ...
             'Position', [140 90 1450 920]);

set(fig, ...
    'DefaultTextFontName', CN_FONT, ...
    'DefaultTextFontSize', FONT_SIZE_NORMAL, ...
    'DefaultTextFontWeight', FONT_WEIGHT_ALL, ...
    'DefaultTextColor', TEXT_COLOR);

ax = axes(fig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
box(ax, 'off');        % 关闭3D外框，只保留坐标轴方向线
view(ax, 3);

set_axes_style(ax, EN_FONT, FONT_SIZE_NORMAL, FONT_WEIGHT_ALL, TEXT_COLOR, AX_LINEWIDTH);

hx = xlabel(ax, '\fontname{Times New Roman}X (mm)', ...
    'Interpreter', 'tex', ...
    'FontName', EN_FONT, ...
    'FontSize', FONT_SIZE_NORMAL, ...
    'FontWeight', FONT_WEIGHT_ALL, ...
    'Color', TEXT_COLOR);

hy = ylabel(ax, '\fontname{Times New Roman}Y (mm)', ...
    'Interpreter', 'tex', ...
    'FontName', EN_FONT, ...
    'FontSize', FONT_SIZE_NORMAL, ...
    'FontWeight', FONT_WEIGHT_ALL, ...
    'Color', TEXT_COLOR);

hz = zlabel(ax, '\fontname{Times New Roman}Z (mm)', ...
    'Interpreter', 'tex', ...
    'FontName', EN_FONT, ...
    'FontSize', FONT_SIZE_NORMAL, ...
    'FontWeight', FONT_WEIGHT_ALL, ...
    'Color', TEXT_COLOR);

ht = title(ax, ['\fontname{' CN_FONT '}残腔上所有切片平面'], ...
    'Interpreter', 'tex', ...
    'FontName', CN_FONT, ...
    'FontSize', FONT_SIZE_NORMAL, ...
    'FontWeight', FONT_WEIGHT_ALL, ...
    'Color', TEXT_COLOR);

% 残腔表面
trisurf(slice_result.F, ...
    slice_result.V(:,1), ...
    slice_result.V(:,2), ...
    slice_result.V(:,3), ...
    'Parent', ax, ...
    'FaceColor', SURFACE_COLOR, ...
    'EdgeColor', 'none', ...
    'FaceAlpha', 0.18);

% 参考轴线（修正后的有效显示段）
h_axis = plot3(ax, ...
    [slice_result.axis_display_p0(1), slice_result.axis_display_p1(1)], ...
    [slice_result.axis_display_p0(2), slice_result.axis_display_p1(2)], ...
    [slice_result.axis_display_p0(3), slice_result.axis_display_p1(3)], ...
    '-', ...
    'Color', STRAIGHT_AXIS_COLOR, ...
    'LineWidth', 3.0);

% 定向包围盒线框
C = slice_result.bbox_world_corners;
E = slice_result.bbox_edges;

h_bbox = [];

for i = 1:size(E,1)
    p1 = C(E(i,1), :);
    p2 = C(E(i,2), :);

    h_tmp = plot3(ax, ...
        [p1(1), p2(1)], ...
        [p1(2), p2(2)], ...
        [p1(3), p2(3)], ...
        '-', ...
        'Color', BBOX_COLOR, ...
        'LineWidth', 1.5);

    if i == 1
        h_bbox = h_tmp;
    end
end

% 所有切片平面
h_first_slice = [];
h_other_slice = [];

for k = 1:num_slices
    P = slice_planes_world{k};

    if k == 1
        h_first_slice = fill3(ax, ...
            P(:,1), P(:,2), P(:,3), ...
            FIRST_SLICE_PLANE_COLOR, ...
            'FaceAlpha', 0.28, ...
            'EdgeColor', FIRST_SLICE_PLANE_COLOR, ...
            'LineWidth', 1.4);
    else
        h_tmp = fill3(ax, ...
            P(:,1), P(:,2), P(:,3), ...
            SLICE_PLANE_COLOR, ...
            'FaceAlpha', 0.20, ...
            'EdgeColor', SLICE_PLANE_COLOR, ...
            'LineWidth', 1.0);

        if isempty(h_other_slice)
            h_other_slice = h_tmp;
        end
    end
end

% 图例
if isempty(h_other_slice)
    lgd = legend(ax, ...
        [h_first_slice, h_axis, h_bbox], ...
        {'第一个切片平面', '参考轴线', '定向包围盒'}, ...
        'Location', 'bestoutside', ...
        'Box', 'off');
else
    lgd = legend(ax, ...
        [h_first_slice, h_other_slice, h_axis, h_bbox], ...
        {'第一个切片平面', '其余切片平面', '参考轴线', '定向包围盒'}, ...
        'Location', 'bestoutside', ...
        'Box', 'off');
end

set_legend_style(lgd, CN_FONT, FONT_SIZE_NORMAL, FONT_WEIGHT_ALL, TEXT_COLOR);

camlight(ax, 'headlight');
lighting(ax, 'gouraud');

% 最后再次强制文字与坐标轴为纯黑，并关闭外框
force_text_black(ax, TEXT_COLOR);

hx.Color = TEXT_COLOR;
hy.Color = TEXT_COLOR;
hz.Color = TEXT_COLOR;
ht.Color = TEXT_COLOR;

fprintf('切片平面可视化完成。\n');

end

%% =========================================================
% 世界坐标 -> 局部坐标
%% =========================================================
function local_pts = world_to_local(P, o, u, v, a)
    X = P - o;
    local_pts = [X * u(:), X * v(:), X * a(:)];
end

%% =========================================================
% 局部坐标 -> 世界坐标
%% =========================================================
function world_pts = local_to_world(P_local, o, u, v, a)
    R = [u(:), v(:), a(:)];
    world_pts = (R * P_local.').';
    world_pts = world_pts + o;
end

%% =========================================================
% 坐标轴样式统一设置
%% =========================================================
function set_axes_style(ax, en_font, font_size, font_weight, text_color, ax_linewidth)
    set(ax, ...
        'FontName', en_font, ...
        'FontSize', font_size, ...
        'FontWeight', font_weight, ...
        'LineWidth', ax_linewidth, ...
        'XColor', text_color, ...
        'YColor', text_color, ...
        'ZColor', text_color, ...
        'Box', 'off');

    ax.XAxis.FontName = en_font;
    ax.YAxis.FontName = en_font;
    ax.ZAxis.FontName = en_font;

    ax.XAxis.FontWeight = font_weight;
    ax.YAxis.FontWeight = font_weight;
    ax.ZAxis.FontWeight = font_weight;

    ax.XAxis.Color = text_color;
    ax.YAxis.Color = text_color;
    ax.ZAxis.Color = text_color;
end

%% =========================================================
% 图例样式统一设置
%% =========================================================
function set_legend_style(lgd, cn_font, font_size, font_weight, text_color)
    set(lgd, ...
        'FontName', cn_font, ...
        'FontSize', font_size, ...
        'FontWeight', font_weight, ...
        'Interpreter', 'none');

    if isprop(lgd, 'TextColor')
        lgd.TextColor = text_color;
    end

    lgd.Color = 'none';
end

%% =========================================================
% 强制文字对象为纯黑
%% =========================================================
function force_text_black(ax, text_color)
    text_objs = findall(ax, 'Type', 'text');

    for k = 1:numel(text_objs)
        text_objs(k).Color = text_color;
    end

    ax.XColor = text_color;
    ax.YColor = text_color;
    ax.ZColor = text_color;

    ax.XAxis.Color = text_color;
    ax.YAxis.Color = text_color;
    ax.ZAxis.Color = text_color;

    ax.Box = 'off';
end