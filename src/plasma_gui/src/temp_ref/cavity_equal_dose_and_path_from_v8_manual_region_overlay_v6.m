
function path_result = cavity_equal_dose_and_path_from_v8_manual_region_overlay_v6()
% =========================================================
% 基于 v8 B样条拟合结果的等剂量面与喷杆路径点生成
% （手动分区角度叠加 + 等剂量面按角度范围着色版）
%
% 功能：
% 1) 调用 cavity_section_contour_bspline_fit_v8()
% 2) 生成等剂量面与喷杆路径点
% 3) 读取 TXT 分区点云，并按模型参考轴线建立局部坐标系
% 4) 将 TXT 点云整体沿参考轴线方向投影到中间层切片平面
% 5) 用手动输入的角度范围 + 0°偏移：
%    a) 对等剂量面按周向角分区着色
%    b) 在投影分区平面中叠加圆弧分区环与边界线
%
% 说明：
% - 不做自动角度扫描分区
% - 分区角度来自用户手动输入
% - 0° 偏移同时作用于：
%   1) 等剂量面着色
%   2) 投影分区平面的圆弧叠加
%
% 单位：
% - 假定 cavity_section_contour_bspline_fit_v8() 输出世界坐标单位为 mm
% - TXT 点云若为 m，则在本文件中统一转换到 mm
% =========================================================

%% =========================
% 用户参数区
%% =========================
NOZZLE_VERTICAL_LENGTH = 5.0;      % 喷嘴竖直长度，单位：mm
SPRAY_ROD_RADIUS = 2.0;            % 喷杆外半径，单位：mm

THETA_SAMPLES_CLOSED = 180;        % 闭合层周向采样数
THETA_SAMPLES_OPEN = 120;          % 开口层周向采样数

% 层间连接
DEFAULT_CONNECTION_REF_ANGLE_DEG = 0.0;
USE_TOP_OPEN_LAYER_ENDPOINT_AS_CONNECTION_REF = true;
OPEN_LAYER_CONNECTION_ENDPOINT_MODE = 'end';   % 'start' 或 'end'

% TXT 分区点云
USE_REGION_TXT = true;
script_dir = fileparts(mfilename('fullpath'));
REGION_TXT_FILE = fullfile(script_dir, 'u1.txt');
REGION_TXT_COORD_SCALE = 1000;   % TXT 点云坐标缩放：m -> mm

% =========================
% 手动分区角度宏
% 每一行表示一个分区角度范围 [起始角, 终止角]，单位：deg
% 若起始角 > 终止角，则表示跨越 360°，例如 [357, 17.8]
% =========================
MANUAL_REGION_ANGLE_RANGES = [ ...
    357.0,  17.8; ...
     17.8,  89.3; ...
     89.3, 113.7; ...
    113.7, 198.6; ...
    198.6, 249.7; ...
    249.7, 357.0  ...
];

% 0° 偏移宏
% 实际使用角度 = 手动角度 + MANUAL_ZERO_DEG_OFFSET
MANUAL_ZERO_DEG_OFFSET = 175.0;

% 视角与角度增长方向宏
% MANUAL_ANGLE_VIEW_DIRECTION:
%   '+a' : 沿参考轴线 +a 方向看向切平面
%   '-a' : 沿参考轴线 -a 方向看向切平面
% MANUAL_ANGLE_INCREASE_DIRECTION:
%   'ccw': 角度按逆时针增长
%   'cw' : 角度按顺时针增长
MANUAL_ANGLE_VIEW_DIRECTION = '-a';
MANUAL_ANGLE_INCREASE_DIRECTION = 'ccw';

% 投影界面绘制参数
SHOW_PROJECTED_POINTS = true;
PROJECTED_POINT_SIZE = 10;
BOUNDARY_LINE_WIDTH = 1.8;
OUTER_ARC_LINE_WIDTH = 4.5;
BOUNDARY_LINE_LENGTH_SCALE = 1.22;
ARC_RADIUS_SCALE = 1.12;
ARC_NUM_POINTS = 200;
SHOW_ANGLE_TEXT = true;
ANGLE_TEXT_RADIUS_SCALE = 1.28;

% 其余可视化
SHOW_SURFACE = true;
SHOW_EQUAL_DOSE_SURFACE = true;
SHOW_STANDALONE_OVERVIEW = true;   % 是否单独弹出“总览”窗口
SHOW_PATH_POINTS = true;
SHOW_PATH_LINES = true;
SHOW_CONNECTION_LINES = true;
SHOW_REFERENCE_POINTS = false;
PATH_POINT_SIZE = 18;
REFERENCE_POINT_SIZE = 14;
PATH_LINE_WIDTH = 2.2;
CONNECTION_LINE_WIDTH = 2.6;

% 字体宏
FONT_CN = 'SimSun';
FONT_EN = 'Times New Roman';
FONT_SIZE = 14;

% 颜色
SURFACE_COLOR = [0.82 0.82 0.82];
DEFAULT_EQUAL_DOSE_SURFACE_COLOR = [0.20 0.75 0.95];
PATH_POINT_COLOR = [0.90 0.15 0.15];
PATH_LINE_COLOR = [0.85 0.10 0.10];
CONNECTION_LINE_COLOR = [0.10 0.65 0.10];
REFERENCE_POINT_COLOR = [0.85 0.55 0.05];
AXIS_COLOR = [0.85 0.0 0.85];

%% =========================
% 1) 获取 v8 拟合结果
%% =========================
fprintf('============================================================\n');
fprintf('基于 v8 拟合结果生成等剂量面与喷杆路径点（手动角度着色版）\n');
fprintf('首先调用 cavity_section_contour_bspline_fit_v8() ...\n');
fprintf('============================================================\n');

fig_before = findall(0, 'Type', 'figure');
bspline_result = cavity_section_contour_bspline_fit_v8();
fig_after = findall(0, 'Type', 'figure');
new_figs = setdiff(fig_after, fig_before);
for i = 1:numel(new_figs)
    if isvalid(new_figs(i))
        close(new_figs(i));
    end
end

F = bspline_result.F;
V = bspline_result.V;
origin_o = bspline_result.origin_o;
u_axis = bspline_result.u_axis;
v_axis = bspline_result.v_axis;
a_axis = bspline_result.a_axis;

num_slices = numel(bspline_result.fitted_curve_local);
fprintf('拟合层数：%d\n', num_slices);
fprintf('喷嘴竖直长度：%.4f mm\n', NOZZLE_VERTICAL_LENGTH);
fprintf('喷杆外半径：%.4f mm\n', SPRAY_ROD_RADIUS);

%% =========================
% 2) 有效层与层间连接参考角
%% =========================
valid_layer_idx = [];
for k = 1:num_slices
    if bspline_result.fit_success(k) && ~isempty(bspline_result.fitted_curve_local{k})
        valid_layer_idx(end+1) = k; %#ok<AGROW>
    end
end

if isempty(valid_layer_idx)
    error('没有有效拟合层。');
end

top_valid_k = valid_layer_idx(1);
mid_valid_k = valid_layer_idx(ceil(numel(valid_layer_idx)/2));
a_mid = mean(bspline_result.fitted_curve_local{mid_valid_k}(:,3));

connection_ref_angle_deg = DEFAULT_CONNECTION_REF_ANGLE_DEG;
if USE_TOP_OPEN_LAYER_ENDPOINT_AS_CONNECTION_REF && ~bspline_result.is_closed_layer(top_valid_k)
    curve_top = bspline_result.fitted_curve_local{top_valid_k};
    if strcmpi(OPEN_LAYER_CONNECTION_ENDPOINT_MODE, 'start')
        pt_ref = curve_top(1, 1:2);
    else
        pt_ref = curve_top(end, 1:2);
    end
    connection_ref_angle_deg = wrap_to_360(atan2d(pt_ref(2), pt_ref(1)));
    fprintf('采用最靠近开口的开口层端点作为整体层间连接参考角：%.3f°\n', connection_ref_angle_deg);
else
    fprintf('采用默认整体层间连接参考角：%.3f°\n', connection_ref_angle_deg);
end

%% =========================
% 3) 读取 TXT 并投影到中间层切片平面
%% =========================
projection_info = struct();
projection_info.enabled = false;
projection_info.projected_local = [];
projection_info.projected_labels = [];
projection_info.a_ref = a_mid;
projection_info.axis_center_local = [NaN, NaN, NaN];
projection_info.axis_center_uv = [NaN, NaN];

if USE_REGION_TXT
    projection_info = load_and_project_region_txt( ...
        REGION_TXT_FILE, REGION_TXT_COORD_SCALE, ...
        origin_o, u_axis, v_axis, a_axis, a_mid);
end

%% =========================
% 4) 手动分区角度信息整理
%% =========================
manual_region_info = build_manual_region_info( ...
    MANUAL_REGION_ANGLE_RANGES, MANUAL_ZERO_DEG_OFFSET, ...
    MANUAL_ANGLE_VIEW_DIRECTION, MANUAL_ANGLE_INCREASE_DIRECTION);

fprintf('手动分区数量：%d\n', size(MANUAL_REGION_ANGLE_RANGES, 1));
fprintf('0° 偏移：%.3f°\n', MANUAL_ZERO_DEG_OFFSET);
fprintf('视角方向：%s\n', MANUAL_ANGLE_VIEW_DIRECTION);
fprintf('角度增长方向：%s\n', MANUAL_ANGLE_INCREASE_DIRECTION);
for i = 1:size(manual_region_info.ranges_deg,1)
    rg = manual_region_info.ranges_deg(i,:);
    fprintf('  手动分区 %d: [%.3f°, %.3f°]%s\n', ...
        i, rg(1), rg(2), ternary_str(rg(1)>rg(2), ' (跨 360°)', ''));
end

%% =========================
% 5) 逐层生成等剂量面与路径点
%% =========================
layer_theta_deg = cell(num_slices, 1);
layer_manual_region_ids = cell(num_slices, 1);
layer_reference_points_local = cell(num_slices, 1);
layer_reference_points_world = cell(num_slices, 1);
layer_path_points_local = cell(num_slices, 1);
layer_path_points_world = cell(num_slices, 1);
layer_nozzle_top_world = cell(num_slices, 1);
layer_nozzle_bottom_world = cell(num_slices, 1);
layer_surface_vertices = cell(num_slices, 1);
layer_surface_faces = cell(num_slices, 1);
layer_surface_face_region_ids = cell(num_slices, 1);
layer_connection_anchor_world = cell(num_slices, 1);
layer_connection_anchor_theta = nan(num_slices, 1);

for kk = 1:numel(valid_layer_idx)
    k = valid_layer_idx(kk);

    curve_local = bspline_result.fitted_curve_local{k};
    uv = curve_local(:, 1:2);
    ak = mean(curve_local(:, 3));

    if size(uv,1) < 4
        continue;
    end

    theta_deg = atan2d(uv(:,2), uv(:,1));
    theta_deg = wrap_to_360(theta_deg);

    if bspline_result.is_closed_layer(k)
        n_theta = THETA_SAMPLES_CLOSED;
        [theta_query, uv_query] = build_closed_theta_correspondence(uv, theta_deg, n_theta);
    else
        n_theta = THETA_SAMPLES_OPEN;
        [theta_query, uv_query] = build_open_theta_correspondence(uv, theta_deg, n_theta);
    end

    if isempty(uv_query) || size(uv_query,1) < 4
        continue;
    end

    theta_query_mod = wrap_to_360(theta_query(:));
    region_ids_theta = assign_manual_region_ids(theta_query_mod, manual_region_info);

    ref_local = [uv_query, ak * ones(size(uv_query,1),1)];
    ref_world = local_to_world(ref_local, origin_o, u_axis, v_axis, a_axis);

    path_local = zeros(size(ref_local));
    for i = 1:size(ref_local,1)
        dir_uv = uv_query(i, :);
        r = norm(dir_uv);
        if r < 1e-12
            path_local(i,:) = [0, 0, ak];
        else
            dir_uv = dir_uv / r;
            path_local(i,:) = [SPRAY_ROD_RADIUS * dir_uv(1), SPRAY_ROD_RADIUS * dir_uv(2), ak];
        end
    end
    path_world = local_to_world(path_local, origin_o, u_axis, v_axis, a_axis);

    top_local = ref_local;
    bottom_local = ref_local;
    top_local(:,3) = top_local(:,3) + NOZZLE_VERTICAL_LENGTH / 2;
    bottom_local(:,3) = bottom_local(:,3) - NOZZLE_VERTICAL_LENGTH / 2;

    top_world = local_to_world(top_local, origin_o, u_axis, v_axis, a_axis);
    bottom_world = local_to_world(bottom_local, origin_o, u_axis, v_axis, a_axis);

    [surfV, surfF, surfFaceRegionIds] = build_strip_surface_with_manual_regions( ...
        top_world, bottom_world, region_ids_theta, bspline_result.is_closed_layer(k));

    layer_theta_deg{k} = theta_query_mod;
    layer_manual_region_ids{k} = region_ids_theta;
    layer_reference_points_local{k} = ref_local;
    layer_reference_points_world{k} = ref_world;
    layer_path_points_local{k} = path_local;
    layer_path_points_world{k} = path_world;
    layer_nozzle_top_world{k} = top_world;
    layer_nozzle_bottom_world{k} = bottom_world;
    layer_surface_vertices{k} = surfV;
    layer_surface_faces{k} = surfF;
    layer_surface_face_region_ids{k} = surfFaceRegionIds;

    if k == top_valid_k && ~bspline_result.is_closed_layer(k) && USE_TOP_OPEN_LAYER_ENDPOINT_AS_CONNECTION_REF
        if strcmpi(OPEN_LAYER_CONNECTION_ENDPOINT_MODE, 'start')
            layer_connection_anchor_world{k} = path_world(1,:);
            layer_connection_anchor_theta(k) = theta_query_mod(1);
        else
            layer_connection_anchor_world{k} = path_world(end,:);
            layer_connection_anchor_theta(k) = theta_query_mod(end);
        end
    else
        [~, idx_anchor] = min(abs(wrap_to_180(theta_query_mod - connection_ref_angle_deg)));
        layer_connection_anchor_world{k} = path_world(idx_anchor, :);
        layer_connection_anchor_theta(k) = theta_query_mod(idx_anchor);
    end

    fprintf('第 %d 层：参考点=%d，路径点=%d，等剂量面顶点=%d\n', ...
        k, size(ref_world,1), size(path_world,1), size(surfV,1));
end

%% =========================
% 6) 生成层间连接线
%% =========================
connection_segments = zeros(0, 6);
if numel(valid_layer_idx) >= 2
    for i = 1:numel(valid_layer_idx)-1
        k1 = valid_layer_idx(i);
        k2 = valid_layer_idx(i+1);

        p1 = layer_connection_anchor_world{k1};
        p2 = layer_connection_anchor_world{k2};

        if ~isempty(p1) && ~isempty(p2)
            connection_segments(end+1, :) = [p1, p2]; %#ok<AGROW>
        end
    end
end

fprintf('有效层数：%d\n', numel(valid_layer_idx));
fprintf('层间连接段数：%d\n', size(connection_segments,1));

%% =========================
% 7) 结果打包
%% =========================
path_result = bspline_result;
path_result.nozzle_vertical_length = NOZZLE_VERTICAL_LENGTH;
path_result.spray_rod_radius = SPRAY_ROD_RADIUS;
path_result.projection_info = projection_info;
path_result.manual_region_info = manual_region_info;
path_result.connection_ref_angle_deg = connection_ref_angle_deg;
path_result.layer_theta_deg = layer_theta_deg;
path_result.layer_manual_region_ids = layer_manual_region_ids;
path_result.layer_reference_points_local = layer_reference_points_local;
path_result.layer_reference_points_world = layer_reference_points_world;
path_result.layer_path_points_local = layer_path_points_local;
path_result.layer_path_points_world = layer_path_points_world;
path_result.layer_nozzle_top_world = layer_nozzle_top_world;
path_result.layer_nozzle_bottom_world = layer_nozzle_bottom_world;
path_result.layer_surface_vertices = layer_surface_vertices;
path_result.layer_surface_faces = layer_surface_faces;
path_result.layer_surface_face_region_ids = layer_surface_face_region_ids;
path_result.layer_connection_anchor_world = layer_connection_anchor_world;
path_result.layer_connection_anchor_theta = layer_connection_anchor_theta;
path_result.connection_segments = connection_segments;
path_result.valid_layer_idx = valid_layer_idx;

%% =========================
% 8) 可视化
%% =========================
fig = figure('Color', 'w', ...
    'Name', '等剂量面与喷杆路径点生成结果（手动分区角度着色版）', ...
    'Position', [70 40 1600 980]);

tg = uitabgroup(fig);

% ---- 总览 ----
tab0 = uitab(tg, 'Title', '总览');
ax0 = axes(tab0);
hold(ax0, 'on');
axis(ax0, 'equal');
grid(ax0, 'on');
view(ax0, 3);
set_axis_fonts(ax0, FONT_EN, FONT_SIZE);
xlabel(ax0, 'X(mm)', 'FontName', FONT_EN, 'FontSize', FONT_SIZE);
ylabel(ax0, 'Y(mm)', 'FontName', FONT_EN, 'FontSize', FONT_SIZE);
zlabel(ax0, 'Z(mm)', 'FontName', FONT_EN, 'FontSize', FONT_SIZE);
title(ax0, '等剂量面与喷杆路径点总览', 'FontName', FONT_CN, 'FontSize', FONT_SIZE);
draw_overview_scene(ax0, bspline_result, F, V, valid_layer_idx, ...
    layer_surface_vertices, layer_surface_faces, layer_surface_face_region_ids, ...
    manual_region_info, DEFAULT_EQUAL_DOSE_SURFACE_COLOR, ...
    SHOW_SURFACE, SURFACE_COLOR, AXIS_COLOR, ...
    SHOW_EQUAL_DOSE_SURFACE, SHOW_REFERENCE_POINTS, ...
    layer_reference_points_world, REFERENCE_POINT_SIZE, REFERENCE_POINT_COLOR, ...
    SHOW_PATH_POINTS, layer_path_points_world, PATH_POINT_SIZE, PATH_POINT_COLOR, ...
    SHOW_PATH_LINES, PATH_LINE_COLOR, PATH_LINE_WIDTH, ...
    SHOW_CONNECTION_LINES, connection_segments, CONNECTION_LINE_COLOR, CONNECTION_LINE_WIDTH);

lgd0 = legend(ax0, {'残腔表面','参考轴线','轮廓参考点','路径点','单层路径','层间连接'}, ...
    'Location', 'bestoutside');
set_legend_fonts(lgd0, FONT_CN, FONT_SIZE);
camlight(ax0, 'headlight');
lighting(ax0, 'gouraud');

% ---- 独立总览窗口 ----
if SHOW_STANDALONE_OVERVIEW
    fig_overview = figure('Color', 'w', ...
        'Name', '总览（独立窗口）', ...
        'Position', [120 80 1150 860]);
    ax_overview = axes(fig_overview);
    hold(ax_overview, 'on');
    axis(ax_overview, 'equal');
    grid(ax_overview, 'on');
    view(ax_overview, 3);
    set_axis_fonts(ax_overview, FONT_EN, FONT_SIZE);
    xlabel(ax_overview, 'X(mm)', 'FontName', FONT_EN, 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    ylabel(ax_overview, 'Y(mm)', 'FontName', FONT_EN, 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    zlabel(ax_overview, 'Z(mm)', 'FontName', FONT_EN, 'FontSize', FONT_SIZE);
    title(ax_overview, '等剂量面与喷杆路径点总览（独立窗口）', 'FontName', FONT_CN, 'FontSize', FONT_SIZE);

    draw_overview_scene(ax_overview, bspline_result, F, V, valid_layer_idx, ...
        layer_surface_vertices, layer_surface_faces, layer_surface_face_region_ids, ...
        manual_region_info, DEFAULT_EQUAL_DOSE_SURFACE_COLOR, ...
        SHOW_SURFACE, SURFACE_COLOR, AXIS_COLOR, ...
        SHOW_EQUAL_DOSE_SURFACE, SHOW_REFERENCE_POINTS, ...
        layer_reference_points_world, REFERENCE_POINT_SIZE, REFERENCE_POINT_COLOR, ...
        SHOW_PATH_POINTS, layer_path_points_world, PATH_POINT_SIZE, PATH_POINT_COLOR, ...
        SHOW_PATH_LINES, PATH_LINE_COLOR, PATH_LINE_WIDTH, ...
        SHOW_CONNECTION_LINES, connection_segments, CONNECTION_LINE_COLOR, CONNECTION_LINE_WIDTH);

    camlight(ax_overview, 'headlight');
    lighting(ax_overview, 'gouraud');
end

% ---- 投影分区平面 ----
if projection_info.enabled
    tabp = uitab(tg, 'Title', '投影分区平面');
    axp = axes(tabp);
    hold(axp, 'on');
    axis(axp, 'equal');
    grid(axp, 'on');
    set_axis_fonts(axp, FONT_EN, FONT_SIZE);
    xlabel(axp, 'u(mm)', 'FontName', FONT_EN, 'FontSize', FONT_SIZE);
    ylabel(axp, 'v(mm)', 'FontName', FONT_EN, 'FontSize', FONT_SIZE);
    title(axp, '投影点云与手动分区角度叠加结果', 'FontName', FONT_CN, 'FontSize', FONT_SIZE);

    if SHOW_PROJECTED_POINTS
        draw_projected_points(axp, projection_info, PROJECTED_POINT_SIZE);
    end

    ctr_uv = projection_info.axis_center_uv;
    plot(axp, ctr_uv(1), ctr_uv(2), 'p', 'MarkerSize', 12, 'MarkerFaceColor', AXIS_COLOR, 'MarkerEdgeColor', AXIS_COLOR);

    draw_manual_region_overlay(axp, projection_info, manual_region_info, ctr_uv, ...
        BOUNDARY_LINE_LENGTH_SCALE, ARC_RADIUS_SCALE, ARC_NUM_POINTS, ...
        BOUNDARY_LINE_WIDTH, OUTER_ARC_LINE_WIDTH, SHOW_ANGLE_TEXT, ...
        ANGLE_TEXT_RADIUS_SCALE, FONT_EN, FONT_SIZE);

    lgdp = legend(axp, {'投影点云','参考轴线与切平面交点'}, 'Location', 'bestoutside');
    set_legend_fonts(lgdp, FONT_CN, FONT_SIZE);
end

% ---- 每层页 ----
for kk = 1:numel(valid_layer_idx)
    k = valid_layer_idx(kk);

    tabk = uitab(tg, 'Title', sprintf('第%d层', k));
    axk = axes(tabk);
    hold(axk, 'on');
    axis(axk, 'equal');
    grid(axk, 'on');
    view(axk, 3);
    set_axis_fonts(axk, FONT_EN, FONT_SIZE);
    xlabel(axk, 'X(mm)', 'FontName', FONT_EN, 'FontSize', FONT_SIZE);
    ylabel(axk, 'Y(mm)', 'FontName', FONT_EN, 'FontSize', FONT_SIZE);
    zlabel(axk, 'Z(mm)', 'FontName', FONT_EN, 'FontSize', FONT_SIZE);
    title(axk, sprintf('第 %d 层等剂量面与路径点', k), 'FontName', FONT_CN, 'FontSize', FONT_SIZE);

    if SHOW_SURFACE
        trisurf(F, V(:,1), V(:,2), V(:,3), ...
            'Parent', axk, 'FaceColor', SURFACE_COLOR, 'EdgeColor', 'none', 'FaceAlpha', 0.08);
    end

    plot3(axk, [bspline_result.axis_display_p0(1), bspline_result.axis_display_p1(1)], ...
              [bspline_result.axis_display_p0(2), bspline_result.axis_display_p1(2)], ...
              [bspline_result.axis_display_p0(3), bspline_result.axis_display_p1(3)], ...
              '-', 'Color', AXIS_COLOR, 'LineWidth', 2.8);

    if SHOW_EQUAL_DOSE_SURFACE && ~isempty(layer_surface_vertices{k})
        draw_manual_region_colored_surface(axk, ...
            layer_surface_vertices{k}, ...
            layer_surface_faces{k}, ...
            layer_surface_face_region_ids{k}, ...
            manual_region_info, DEFAULT_EQUAL_DOSE_SURFACE_COLOR);
    end

    if SHOW_REFERENCE_POINTS && ~isempty(layer_reference_points_world{k})
        scatter3(axk, layer_reference_points_world{k}(:,1), layer_reference_points_world{k}(:,2), layer_reference_points_world{k}(:,3), ...
            REFERENCE_POINT_SIZE, REFERENCE_POINT_COLOR, 'filled');
    end

    if SHOW_PATH_POINTS && ~isempty(layer_path_points_world{k})
        scatter3(axk, layer_path_points_world{k}(:,1), layer_path_points_world{k}(:,2), layer_path_points_world{k}(:,3), ...
            PATH_POINT_SIZE, PATH_POINT_COLOR, 'filled');
    end

    if SHOW_PATH_LINES && ~isempty(layer_path_points_world{k})
        plot3(axk, layer_path_points_world{k}(:,1), layer_path_points_world{k}(:,2), layer_path_points_world{k}(:,3), ...
            '-', 'Color', PATH_LINE_COLOR, 'LineWidth', PATH_LINE_WIDTH);
    end

    if SHOW_CONNECTION_LINES && ~isempty(layer_connection_anchor_world{k})
        scatter3(axk, layer_connection_anchor_world{k}(1), layer_connection_anchor_world{k}(2), layer_connection_anchor_world{k}(3), ...
            PATH_POINT_SIZE + 20, CONNECTION_LINE_COLOR, 'filled');
    end

    lgdk = legend(axk, {'残腔表面','参考轴线','轮廓参考点','路径点','单层路径','层间连接基准点'}, ...
        'Location', 'bestoutside');
    set_legend_fonts(lgdk, FONT_CN, FONT_SIZE);

    camlight(axk, 'headlight');
    lighting(axk, 'gouraud');
end

fprintf('等剂量面与路径点生成完成。\n');
end

%% =========================================================
% 绘制总览场景（供 tab 与独立窗口复用）
%% =========================================================
function draw_overview_scene(ax, bspline_result, F, V, valid_layer_idx, ...
    layer_surface_vertices, layer_surface_faces, layer_surface_face_region_ids, ...
    manual_region_info, default_equal_dose_surface_color, ...
    show_surface, surface_color, axis_color, ...
    show_equal_dose_surface, show_reference_points, ...
    layer_reference_points_world, reference_point_size, reference_point_color, ...
    show_path_points, layer_path_points_world, path_point_size, path_point_color, ...
    show_path_lines, path_line_color, path_line_width, ...
    show_connection_lines, connection_segments, connection_line_color, connection_line_width)

    if show_surface
        trisurf(F, V(:,1), V(:,2), V(:,3), ...
            'Parent', ax, 'FaceColor', surface_color, 'EdgeColor', 'none', 'FaceAlpha', 0.10);
    end

    plot3(ax, [bspline_result.axis_display_p0(1), bspline_result.axis_display_p1(1)], ...
              [bspline_result.axis_display_p0(2), bspline_result.axis_display_p1(2)], ...
              [bspline_result.axis_display_p0(3), bspline_result.axis_display_p1(3)], ...
              '-', 'Color', axis_color, 'LineWidth', 3.0);

    for kk = 1:numel(valid_layer_idx)
        k = valid_layer_idx(kk);

        if show_equal_dose_surface && ~isempty(layer_surface_vertices{k})
            draw_manual_region_colored_surface(ax, ...
                layer_surface_vertices{k}, ...
                layer_surface_faces{k}, ...
                layer_surface_face_region_ids{k}, ...
                manual_region_info, default_equal_dose_surface_color);
        end

        if show_reference_points && ~isempty(layer_reference_points_world{k})
            scatter3(ax, layer_reference_points_world{k}(:,1), layer_reference_points_world{k}(:,2), layer_reference_points_world{k}(:,3), ...
                reference_point_size, reference_point_color, 'filled');
        end

        if show_path_points && ~isempty(layer_path_points_world{k})
            scatter3(ax, layer_path_points_world{k}(:,1), layer_path_points_world{k}(:,2), layer_path_points_world{k}(:,3), ...
                path_point_size, path_point_color, 'filled');
        end

        if show_path_lines && ~isempty(layer_path_points_world{k})
            plot3(ax, layer_path_points_world{k}(:,1), layer_path_points_world{k}(:,2), layer_path_points_world{k}(:,3), ...
                '-', 'Color', path_line_color, 'LineWidth', path_line_width);
        end
    end

    if show_connection_lines && ~isempty(connection_segments)
        for i = 1:size(connection_segments,1)
            seg = connection_segments(i,:);
            plot3(ax, seg([1 4]), seg([2 5]), seg([3 6]), '-', ...
                'Color', connection_line_color, 'LineWidth', connection_line_width);
        end
    end
end

%% =========================================================
% 读取并投影 TXT 点云
%% =========================================================
function projection_info = load_and_project_region_txt(txt_file, coord_scale, o, u, v, a, a_ref)
    projection_info = struct();
    projection_info.enabled = false;

    if ~exist(txt_file, 'file')
        warning('分区 TXT 文件不存在：%s', txt_file);
        return;
    end

    data = load(txt_file);
    if size(data,2) < 4
        warning('分区 TXT 至少需要 4 列：x y z label');
        return;
    end

    xyz_world = data(:,1:3) * coord_scale;
    labels = data(:,4);

    P_local = world_to_local(xyz_world, o, u, v, a);
    P_proj = P_local;
    P_proj(:,3) = a_ref;

    axis_center_world = o + a_ref * a(:)';
    axis_center_local = world_to_local(axis_center_world, o, u, v, a);

    projection_info.enabled = true;
    projection_info.projected_local = P_proj;
    projection_info.projected_labels = labels;
    projection_info.axis_center_local = axis_center_local;
    projection_info.axis_center_uv = axis_center_local(1,1:2);
    projection_info.unique_labels = unique(labels).';
    cmap = lines(max(numel(projection_info.unique_labels), 7));
    projection_info.colors = cmap(1:numel(projection_info.unique_labels), :);
    projection_info.a_ref = a_ref;
end

%% =========================================================
% 手动分区角度信息
%% =========================================================
function manual_region_info = build_manual_region_info(angle_ranges, zero_offset, view_direction, increase_direction)
    if size(angle_ranges,2) ~= 2
        error('MANUAL_REGION_ANGLE_RANGES 必须为 N×2 矩阵。');
    end

    n_regions = size(angle_ranges,1);
    shifted = zeros(size(angle_ranges));

    % 总方向是否翻转：
    % 视角 -a 反一次；顺时针 cw 再反一次
    reverse_count = 0;
    if strcmpi(strtrim(view_direction), '-a')
        reverse_count = reverse_count + 1;
    elseif ~strcmpi(strtrim(view_direction), '+a')
        error('MANUAL_ANGLE_VIEW_DIRECTION 只能为 ''+a'' 或 ''-a''。');
    end

    if strcmpi(strtrim(increase_direction), 'cw')
        reverse_count = reverse_count + 1;
    elseif ~strcmpi(strtrim(increase_direction), 'ccw')
        error('MANUAL_ANGLE_INCREASE_DIRECTION 只能为 ''ccw'' 或 ''cw''。');
    end

    direction_reversed = mod(reverse_count, 2) == 1;

    % 对“角度区间”做变换时，若方向翻转，必须交换区间端点
    for i = 1:n_regions
        a0 = angle_ranges(i,1);
        a1 = angle_ranges(i,2);

        a0_t = transform_single_angle(a0, zero_offset, view_direction, increase_direction);
        a1_t = transform_single_angle(a1, zero_offset, view_direction, increase_direction);

        if direction_reversed
            shifted(i,:) = [a1_t, a0_t];
        else
            shifted(i,:) = [a0_t, a1_t];
        end
    end

    manual_region_info = struct();
    manual_region_info.ranges_deg = shifted;
    manual_region_info.zero_offset = zero_offset;
    manual_region_info.view_direction = view_direction;
    manual_region_info.increase_direction = increase_direction;
    manual_region_info.direction_reversed = direction_reversed;
    manual_region_info.num_regions = n_regions;
    cmap = lines(max(n_regions, 7));
    manual_region_info.colors = cmap(1:n_regions, :);
end

function theta_out = transform_single_angle(theta_in, zero_offset, view_direction, increase_direction)
    theta_out = wrap_to_360(theta_in);

    % 先根据观察视角决定平面角度方向
    if strcmpi(strtrim(view_direction), '-a')
        theta_out = wrap_to_360(-theta_out);
    elseif ~strcmpi(strtrim(view_direction), '+a')
        error('MANUAL_ANGLE_VIEW_DIRECTION 只能为 ''+a'' 或 ''-a''。');
    end

    % 再根据顺/逆时针增长方式调整
    if strcmpi(strtrim(increase_direction), 'cw')
        theta_out = wrap_to_360(-theta_out);
    elseif ~strcmpi(strtrim(increase_direction), 'ccw')
        error('MANUAL_ANGLE_INCREASE_DIRECTION 只能为 ''ccw'' 或 ''cw''。');
    end

    % 最后加 0° 偏移
    theta_out = wrap_to_360(theta_out + zero_offset);
end

%% =========================================================
% 将角度映射到手动分区 ID
%% =========================================================
function region_ids = assign_manual_region_ids(theta_deg, manual_region_info)
    theta_deg = wrap_to_360(theta_deg(:));
    region_ids = nan(numel(theta_deg),1);

    for i = 1:numel(theta_deg)
        th = theta_deg(i);
        matched = false;
        for r = 1:size(manual_region_info.ranges_deg,1)
            a0 = manual_region_info.ranges_deg(r,1);
            a1 = manual_region_info.ranges_deg(r,2);
            if angle_in_interval(th, a0, a1)
                region_ids(i) = r;
                matched = true;
                break;
            end
        end
        if ~matched
            % 若刚好落在数值边界外，采用最近分区中心
            centers = zeros(size(manual_region_info.ranges_deg,1),1);
            for r = 1:size(manual_region_info.ranges_deg,1)
                centers(r) = interval_center_deg(manual_region_info.ranges_deg(r,1), manual_region_info.ranges_deg(r,2));
            end
            [~, idx] = min(abs(wrap_to_180(th - centers)));
            region_ids(i) = idx;
        end
    end
end

function tf = angle_in_interval(th, a0, a1)
    th = wrap_to_360(th);
    a0 = wrap_to_360(a0);
    a1 = wrap_to_360(a1);
    if a0 <= a1
        tf = (th >= a0) && (th < a1);
    else
        tf = (th >= a0) || (th < a1);
    end
end

function c = interval_center_deg(a0, a1)
    a0 = wrap_to_360(a0);
    a1 = wrap_to_360(a1);
    d = wrap_to_360(a1 - a0);
    c = wrap_to_360(a0 + d/2);
end

%% =========================================================
% 在投影平面上绘制手动分区叠加
%% =========================================================
function draw_manual_region_overlay(ax, projection_info, manual_region_info, center_uv, ...
    boundary_line_length_scale, arc_radius_scale, arc_num_points, ...
    boundary_line_width, outer_arc_line_width, show_angle_text, ...
    angle_text_radius_scale, font_en, font_size)

    if isempty(projection_info.projected_local)
        return;
    end

    P = projection_info.projected_local;
    dx = P(:,1) - center_uv(1);
    dy = P(:,2) - center_uv(2);
    r_all = sqrt(dx.^2 + dy.^2);
    r_max = max(r_all);
    if r_max < 1e-9
        r_max = 1;
    end

    line_radius = boundary_line_length_scale * r_max;
    arc_radius = arc_radius_scale * r_max;
    text_radius = angle_text_radius_scale * r_max;
    cx = center_uv(1);
    cy = center_uv(2);

    ranges = manual_region_info.ranges_deg;
    colors = manual_region_info.colors;
    n_regions = size(ranges,1);

    % 先画彩色圆弧
    for i = 1:n_regions
        a0 = ranges(i,1);
        a1 = ranges(i,2);

        theta_arc = build_arc_angles_deg(a0, a1, arc_num_points);
        x_arc = cx + arc_radius * cosd(theta_arc);
        y_arc = cy + arc_radius * sind(theta_arc);

        plot(ax, x_arc, y_arc, '-', 'Color', colors(i,:), 'LineWidth', outer_arc_line_width);
    end

    % 再画边界径向线
    boundary_angles = unique(ranges(:));
    for i = 1:numel(boundary_angles)
        ang = boundary_angles(i);
        x_line = [cx, cx + line_radius * cosd(ang)];
        y_line = [cy, cy + line_radius * sind(ang)];
        plot(ax, x_line, y_line, '-', 'Color', [0 0 0], 'LineWidth', boundary_line_width);

        if show_angle_text
            xt = cx + text_radius * cosd(ang);
            yt = cy + text_radius * sind(ang);
            text(ax, xt, yt, sprintf('%.1f°', ang), ...
                'FontName', font_en, 'FontSize', font_size, ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
        end
    end
end

%% =========================================================
% 根据起止角构造圆弧角度序列（处理跨 360°）
%% =========================================================
function theta_arc = build_arc_angles_deg(a0, a1, n_pts)
    a0 = wrap_to_360(a0);
    a1 = wrap_to_360(a1);

    if a0 <= a1
        theta_arc = linspace(a0, a1, n_pts);
    else
        theta_arc = linspace(a0, a1 + 360, n_pts);
    end
    theta_arc = wrap_to_360(theta_arc);
end

%% =========================================================
% 绘制投影点云
%% =========================================================
function draw_projected_points(ax, projection_info, point_size)
    P = projection_info.projected_local;
    labels = projection_info.projected_labels;

    for i = 1:numel(projection_info.unique_labels)
        lb = projection_info.unique_labels(i);
        idx = labels == lb;
        scatter(ax, P(idx,1), P(idx,2), point_size, projection_info.colors(i,:), 'filled');
    end
end

%% =========================================================
% 绘制手动分区着色的等剂量面
%% =========================================================
function draw_manual_region_colored_surface(ax, Vsurf, Fsurf, faceRegionIds, manual_region_info, default_color)
    if isempty(Vsurf) || isempty(Fsurf)
        return;
    end

    if isempty(faceRegionIds)
        patch(ax, 'Vertices', Vsurf, 'Faces', Fsurf, ...
            'FaceColor', default_color, 'FaceAlpha', 0.20, 'EdgeColor', 'none');
        return;
    end

    uniq = unique(faceRegionIds(:)).';
    for i = 1:numel(uniq)
        rid = uniq(i);
        idxf = find(faceRegionIds == rid);
        if isempty(idxf)
            continue;
        end

        c = default_color;
        if rid >= 1 && rid <= size(manual_region_info.colors,1)
            c = manual_region_info.colors(rid,:);
        end

        patch(ax, 'Vertices', Vsurf, 'Faces', Fsurf(idxf,:), ...
            'FaceColor', c, 'FaceAlpha', 0.20, 'EdgeColor', 'none');
    end
end

%% =========================================================
% 闭合层：建立周期性角度对应关系
%% =========================================================
function [theta_query, uv_query] = build_closed_theta_correspondence(uv, theta_deg, n_theta)
    theta_deg = wrap_to_360(theta_deg(:));
    [theta_sorted, ord] = sort(theta_deg, 'ascend');
    uv_sorted = uv(ord, :);

    [theta_unique, ia] = unique(round(theta_sorted, 8), 'stable');
    uv_unique = uv_sorted(ia, :);

    if numel(theta_unique) < 4
        theta_query = [];
        uv_query = [];
        return;
    end

    theta_ext = [theta_unique; theta_unique(1) + 360];
    uv_ext = [uv_unique; uv_unique(1,:)];

    theta_query = linspace(0, 360, n_theta + 1).';
    theta_query(end) = [];

    u_query = interp1(theta_ext, uv_ext(:,1), theta_query, 'pchip');
    v_query = interp1(theta_ext, uv_ext(:,2), theta_query, 'pchip');
    uv_query = [u_query, v_query];
end

%% =========================================================
% 开口层：建立非周期角度对应关系
%% =========================================================
function [theta_query, uv_query] = build_open_theta_correspondence(uv, theta_deg, n_theta)
    theta_deg = theta_deg(:);
    theta_unwrap = unwrap(theta_deg * pi / 180) * 180 / pi;

    if theta_unwrap(end) < theta_unwrap(1)
        theta_unwrap = flipud(theta_unwrap);
        uv = flipud(uv);
    end

    [theta_unique, ia] = unique(round(theta_unwrap, 8), 'stable');
    uv_unique = uv(ia, :);

    if numel(theta_unique) < 4
        theta_query = [];
        uv_query = [];
        return;
    end

    theta_query = linspace(theta_unique(1), theta_unique(end), n_theta).';
    u_query = interp1(theta_unique, uv_unique(:,1), theta_query, 'pchip');
    v_query = interp1(theta_unique, uv_unique(:,2), theta_query, 'pchip');
    uv_query = [u_query, v_query];
end

%% =========================================================
% 根据上下边界构造带状曲面 + 分区 ID
%% =========================================================
function [Vsurf, Fsurf, faceRegionIds] = build_strip_surface_with_manual_regions(top_pts, bottom_pts, pointRegionIds, is_closed)
    n = size(top_pts,1);
    Vsurf = [top_pts; bottom_pts];
    Fsurf = zeros(0,4);
    faceRegionIds = zeros(0,1);

    if n < 2
        return;
    end

    if is_closed
        max_i = n;
    else
        max_i = n - 1;
    end

    for i = 1:max_i
        i2 = i + 1;
        if i == n
            i2 = 1;
        end

        f = [i, i2, n + i2, n + i];
        Fsurf(end+1, :) = f; %#ok<AGROW>
        faceRegionIds(end+1,1) = pointRegionIds(i); %#ok<AGROW>
    end
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
% 角度工具
%% =========================================================
function th = wrap_to_360(th)
    th = mod(th, 360);
    th(th < 0) = th(th < 0) + 360;
end

function th = wrap_to_180(th)
    th = mod(th + 180, 360) - 180;
end

%% =========================================================
% 坐标轴字体
%% =========================================================
function set_axis_fonts(ax, font_en, font_size)
    set(ax, 'FontName', font_en, 'FontSize', font_size);
end

%% =========================================================
% 图例字体
%% =========================================================
function set_legend_fonts(lgd, font_cn, font_size)
    try
        lgd.FontSize = font_size;
        lgd.FontName = font_cn;
    catch
        set(lgd, 'FontSize', font_size);
    end
end

%% =========================================================
% 小工具：三元字符串
%% =========================================================
function s = ternary_str(cond, s1, s2)
    if cond
        s = s1;
    else
        s = s2;
    end
end
