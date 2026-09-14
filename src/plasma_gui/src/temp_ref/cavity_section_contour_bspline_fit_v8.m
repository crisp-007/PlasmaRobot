
function bspline_result = cavity_section_contour_bspline_fit_v8()
% =========================================================
% 截面轮廓候选点的 B 样条拟合（第八版）
%
% 核心思路：
% 1) 闭合层：
%    - 候选点角度覆盖率判定为闭合层
%    - 角度分箱代表化
%    - 预平滑
%    - 闭合 B 样条拟合
%
% 2) 开口层：
%    - 通过周向角排序得到首端到尾端的有序候选点
%    - 角度分箱代表化
%    - 预平滑
%    - 开放 B 样条拟合
%    - 用较密拟合点表示拟合曲线
%    - 对首尾端拟合点做曲率检测
%    - 若首尾端出现局部大曲率抖动，则删除该段异常点
%    - 直接连接剩余拟合点作为最终拟合线
%
% 可视化要求：
% - 中文字体：宋体
% - 英文与数字字体：Times New Roman
% - 默认字号：12
% - 坐标轴单位：mm
% =========================================================

%% -------------------------
% 用户参数区
%% -------------------------
FIT_NUM_POINTS = 320;            % 每层拟合曲线输出点数（越大越密）
USE_CS_CLOSED = true;            % 闭合层优先使用 cscvn
SHOW_CANDIDATE_POINTS = false;   % 每层页面是否显示原始候选点
SHOW_BIN_REP_POINTS = true;      % 是否显示分箱后的代表点
SHOW_OPEN_ORDERED_POINTS = true; % 是否显示开口层有序点
CANDIDATE_POINT_SIZE = 18;
REP_POINT_SIZE = 26;
ORDERED_POINT_SIZE = 20;
CURVE_LINE_WIDTH = 2.4;

% 闭合判据：角度覆盖率
COVERAGE_THRESHOLD = 0.80;       % 角度覆盖率阈值，超过则视为闭合层

% 平滑参数宏
ENABLE_PRE_SMOOTH = true;        % 是否对角度分箱代表点做预平滑
SMOOTH_WINDOW = 7;               % 移动平均窗口大小，建议奇数，>=3；越大越平滑

% 角度分箱代表化参数
FIT_ANGLE_BIN_DEG = 1.0;         % 拟合前角度分箱宽度（度）
MIN_BIN_POINT_COUNT = 1;         % 一个分箱中最少需要的点数；默认1表示有点就参与

% 开口层端部曲率清理参数
ENABLE_END_CURVATURE_TRIM = true;   % 是否启用首尾端曲率清理
END_CHECK_COUNT =500;               % 首尾各检查多少个拟合点
CURVATURE_PEAK_RATIO = 1;%1.0         % 端部曲率峰值相对主体曲率中位数的倍数阈值
CURVATURE_RECOVER_RATIO = 0.01;      % 曲率恢复正常的判据
MAX_TRIM_COUNT_PER_END = 50;        % 每一端最多删除多少个拟合点
MIN_KEEP_POINT_COUNT = 60;          % 清理后最少保留点数，防止删太多

% 字体宏
FONT_CN = 'SimSun';                 % 中文：宋体
FONT_EN = 'Times New Roman';        % 英文/数字：Times New Roman
FONT_SIZE = 12;                     % 默认字号

% 独立弹窗字体宏（仅用于新增 figure）
POPUP_FONT_CN = 'SimSun';           % 中文：宋体
POPUP_FONT_EN = 'Times New Roman';  % 英文/数字：Times New Roman
POPUP_FONT_SIZE = 18;               % 统一字号

% 颜色
SURFACE_COLOR = [0.82 0.82 0.82];
REFERENCE_AXIS_COLOR = [0.85 0.0 0.85];
BBOX_COLOR = [0.10 0.70 0.20];
FIRST_SLICE_PLANE_COLOR = [1.00 0.00 0.00];
SLICE_PLANE_COLOR = [0.25 0.60 1.00];
CANDIDATE_POINT_COLOR = [0.00 0.00 0.00];
REP_POINT_COLOR = [0.75 0.00 0.75];
ORDERED_POINT_COLOR = [0.10 0.60 0.10];
BSPLINE_CURVE_COLOR = [0.00 0.45 0.95];
OPEN_CURVE_COLOR = [0.95 0.45 0.10];

%% -------------------------
% 1) 调用前一步结果
%% -------------------------
fprintf('============================================================\n');
fprintf('截面轮廓候选点的 B 样条拟合（第八版）\n');
fprintf('首先调用 cavity_section_contour_candidates_intersection_v1() ...\n');
fprintf('============================================================\n');

fig_before = findall(0, 'Type', 'figure');
contour_result = cavity_section_contour_candidates_intersection_v1();

fig_after = findall(0, 'Type', 'figure');
new_figs = setdiff(fig_after, fig_before);
for i = 1:numel(new_figs)
    if isvalid(new_figs(i))
        close(new_figs(i));
    end
end

%% -------------------------
% 2) 读取基础数据
%% -------------------------
F = contour_result.F;
V_mesh = contour_result.V;
u_axis = contour_result.u_axis;
v_axis = contour_result.v_axis;
a_axis = contour_result.a_axis;
origin_o = contour_result.origin_o;

num_slices = numel(contour_result.slice_positions);
candidate_points_world = contour_result.candidate_points_world;
candidate_point_angles = contour_result.candidate_point_angles;
slice_positions = contour_result.slice_positions;
angle_step_deg = contour_result.angle_step_deg;

fprintf('总切片层数 = %d\n', num_slices);
fprintf('每层拟合输出点数 = %d\n', FIT_NUM_POINTS);
fprintf('角度覆盖率阈值 = %.3f\n', COVERAGE_THRESHOLD);
fprintf('预平滑开关 = %d, 平滑窗口 = %d\n', ENABLE_PRE_SMOOTH, SMOOTH_WINDOW);
fprintf('拟合角度分箱宽度 = %.3f deg\n', FIT_ANGLE_BIN_DEG);
fprintf('端部曲率清理开关 = %d\n', ENABLE_END_CURVATURE_TRIM);

%% -------------------------
% 3) 对每层候选点做 B 样条拟合
%% -------------------------
fitted_curve_world = cell(num_slices, 1);
fitted_curve_local = cell(num_slices, 1);
sorted_candidate_local = cell(num_slices, 1);
sorted_candidate_world = cell(num_slices, 1);
bin_rep_points_local = cell(num_slices, 1);
bin_rep_points_world = cell(num_slices, 1);
bin_rep_counts = cell(num_slices, 1);
open_ordered_points_local = cell(num_slices, 1);
open_ordered_points_world = cell(num_slices, 1);
curvature_values_per_slice = cell(num_slices, 1);

fit_success = false(num_slices, 1);
is_closed_layer = false(num_slices, 1);
angle_coverage = zeros(num_slices, 1);

for k = 1:num_slices
    P_world = candidate_points_world{k};
    ak = slice_positions(k);
    ang_k = candidate_point_angles{k};

    if isempty(P_world) || size(P_world, 1) < 4
        fprintf('第 %d 层：候选点不足（%d 个），跳过拟合。\n', k, size(P_world,1));
        fitted_curve_world{k} = zeros(0, 3);
        fitted_curve_local{k} = zeros(0, 3);
        sorted_candidate_local{k} = zeros(0, 3);
        sorted_candidate_world{k} = zeros(0, 3);
        bin_rep_points_local{k} = zeros(0, 3);
        bin_rep_points_world{k} = zeros(0, 3);
        bin_rep_counts{k} = [];
        open_ordered_points_local{k} = zeros(0, 3);
        open_ordered_points_world{k} = zeros(0, 3);
        curvature_values_per_slice{k} = [];
        continue;
    end

    P_local = world_to_local(P_world, origin_o, u_axis, v_axis, a_axis);
    uv = P_local(:, 1:2);

    theta_geom = atan2d(uv(:,2), uv(:,1));
    theta_geom(theta_geom < 0) = theta_geom(theta_geom < 0) + 360;
    [theta_sorted, order] = sort(theta_geom, 'ascend');

    uv_sorted = uv(order, :);
    P_world_sorted = P_world(order, :);

    sorted_candidate_local{k} = [uv_sorted, ak * ones(size(uv_sorted,1),1)];
    sorted_candidate_world{k} = P_world_sorted;

    coverage_k = estimate_angle_coverage(ang_k, angle_step_deg);
    angle_coverage(k) = coverage_k;
    is_closed_layer(k) = coverage_k >= COVERAGE_THRESHOLD;

    [uv_rep, rep_counts, ~] = angular_bin_representatives( ...
        uv_sorted, theta_sorted, FIT_ANGLE_BIN_DEG, MIN_BIN_POINT_COUNT, is_closed_layer(k));

    bin_rep_counts{k} = rep_counts;

    if isempty(uv_rep) || size(uv_rep,1) < 4
        fprintf('第 %d 层：分箱代表点不足（%d 个），跳过拟合。\n', k, size(uv_rep,1));
        fitted_curve_world{k} = zeros(0, 3);
        fitted_curve_local{k} = zeros(0, 3);
        bin_rep_points_local{k} = zeros(0, 3);
        bin_rep_points_world{k} = zeros(0, 3);
        open_ordered_points_local{k} = zeros(0, 3);
        open_ordered_points_world{k} = zeros(0, 3);
        curvature_values_per_slice{k} = [];
        continue;
    end

    rep_local3 = [uv_rep, ak * ones(size(uv_rep,1),1)];
    rep_world3 = local_to_world(rep_local3, origin_o, u_axis, v_axis, a_axis);
    bin_rep_points_local{k} = rep_local3;
    bin_rep_points_world{k} = rep_world3;

    uv_fit_input = uv_rep;
    if ENABLE_PRE_SMOOTH
        if is_closed_layer(k)
            uv_fit_input = circular_moving_average(uv_rep, SMOOTH_WINDOW);
        else
            uv_fit_input = open_moving_average(uv_rep, SMOOTH_WINDOW);
        end
    end

    try
        if is_closed_layer(k)
            uv_closed = [uv_fit_input; uv_fit_input(1, :)];
            uv_closed = remove_consecutive_duplicate_points(uv_closed);

            if size(uv_closed, 1) < 4
                error('闭合后有效点不足');
            end

            if USE_CS_CLOSED
                pp = cscvn(uv_closed.');
                t_query = linspace(pp.breaks(1), pp.breaks(end), FIT_NUM_POINTS);
                uv_fit = fnval(pp, t_query).';
            else
                uv_fit = closed_spline_fallback(uv_closed, FIT_NUM_POINTS);
            end

            curv = compute_discrete_curvature(uv_fit);
            curvature_values_per_slice{k} = curv;

        else
            uv_open = remove_consecutive_duplicate_points(uv_fit_input);
            if size(uv_open, 1) < 4
                error('开放曲线有效点不足');
            end

            open_ordered_points_local{k} = [uv_open, ak * ones(size(uv_open,1),1)];
            open_ordered_points_world{k} = local_to_world(open_ordered_points_local{k}, origin_o, u_axis, v_axis, a_axis);

            uv_fit = open_spline_fallback(uv_open, FIT_NUM_POINTS);

            curv = compute_discrete_curvature(uv_fit);
            curvature_values_per_slice{k} = curv;

            if ENABLE_END_CURVATURE_TRIM
                [uv_fit, curv] = trim_open_curve_by_end_curvature( ...
                    uv_fit, curv, END_CHECK_COUNT, CURVATURE_PEAK_RATIO, ...
                    CURVATURE_RECOVER_RATIO, MAX_TRIM_COUNT_PER_END, MIN_KEEP_POINT_COUNT);
                curvature_values_per_slice{k} = curv;
            end
        end

        local_fit = [uv_fit, ak * ones(size(uv_fit,1), 1)];
        world_fit = local_to_world(local_fit, origin_o, u_axis, v_axis, a_axis);

        fitted_curve_local{k} = local_fit;
        fitted_curve_world{k} = world_fit;
        fit_success(k) = true;

        if is_closed_layer(k)
            fprintf('第 %d 层：闭合拟合成功，覆盖率=%.3f，原候选点=%d，代表点=%d，输出点=%d\n', ...
                k, coverage_k, size(uv_sorted,1), size(uv_rep,1), size(world_fit,1));
        else
            fprintf('第 %d 层：开口拟合成功，覆盖率=%.3f，原候选点=%d，代表点=%d，输出点=%d\n', ...
                k, coverage_k, size(uv_sorted,1), size(uv_rep,1), size(world_fit,1));
        end

    catch ME
        fprintf('第 %d 层：拟合失败，原因：%s\n', k, ME.message);
        fitted_curve_local{k} = zeros(0, 3);
        fitted_curve_world{k} = zeros(0, 3);
        curvature_values_per_slice{k} = [];
    end
end

%% -------------------------
% 4) 结果打包
%% -------------------------
bspline_result = contour_result;
bspline_result.fit_num_points = FIT_NUM_POINTS;
bspline_result.fitted_curve_world = fitted_curve_world;
bspline_result.fitted_curve_local = fitted_curve_local;
bspline_result.sorted_candidate_local = sorted_candidate_local;
bspline_result.sorted_candidate_world = sorted_candidate_world;
bspline_result.bin_rep_points_local = bin_rep_points_local;
bspline_result.bin_rep_points_world = bin_rep_points_world;
bspline_result.bin_rep_counts = bin_rep_counts;
bspline_result.open_ordered_points_local = open_ordered_points_local;
bspline_result.open_ordered_points_world = open_ordered_points_world;
bspline_result.curvature_values_per_slice = curvature_values_per_slice;
bspline_result.fit_success = fit_success;
bspline_result.is_closed_layer = is_closed_layer;
bspline_result.angle_coverage = angle_coverage;
bspline_result.coverage_threshold = COVERAGE_THRESHOLD;
bspline_result.enable_pre_smooth = ENABLE_PRE_SMOOTH;
bspline_result.smooth_window = SMOOTH_WINDOW;
bspline_result.fit_angle_bin_deg = FIT_ANGLE_BIN_DEG;
bspline_result.min_bin_point_count = MIN_BIN_POINT_COUNT;

%% -------------------------
% 5) 标签页可视化
%% -------------------------
fig = figure('Color', 'w', ...
    'Name', '截面轮廓候选点的 B 样条拟合（第八版）', ...
    'Position', [60 40 1550 980]);

tg = uitabgroup(fig);

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
title(ax0, '所有层的 B 样条拟合截面轮廓总览', 'FontName', FONT_CN, 'FontSize', FONT_SIZE);

trisurf(F, V_mesh(:,1), V_mesh(:,2), V_mesh(:,3), ...
    'Parent', ax0, ...
    'FaceColor', SURFACE_COLOR, ...
    'EdgeColor', 'none', ...
    'FaceAlpha', 0.12);

plot3(ax0, [contour_result.axis_display_p0(1), contour_result.axis_display_p1(1)], ...
          [contour_result.axis_display_p0(2), contour_result.axis_display_p1(2)], ...
          [contour_result.axis_display_p0(3), contour_result.axis_display_p1(3)], ...
          '-', 'Color', REFERENCE_AXIS_COLOR, 'LineWidth', 3.0);

C = contour_result.bbox_world_corners;
E = contour_result.bbox_edges;
for i = 1:size(E,1)
    p1 = C(E(i,1), :);
    p2 = C(E(i,2), :);
    plot3(ax0, [p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
        '-', 'Color', BBOX_COLOR, 'LineWidth', 1.0);
end

for k = 1:num_slices
    Pk = contour_result.slice_planes_world{k};
    if k == 1
        fill3(ax0, Pk(:,1), Pk(:,2), Pk(:,3), FIRST_SLICE_PLANE_COLOR, ...
            'FaceAlpha', 0.08, ...
            'EdgeColor', FIRST_SLICE_PLANE_COLOR, ...
            'LineWidth', 0.8);
    else
        fill3(ax0, Pk(:,1), Pk(:,2), Pk(:,3), SLICE_PLANE_COLOR, ...
            'FaceAlpha', 0.05, ...
            'EdgeColor', SLICE_PLANE_COLOR, ...
            'LineWidth', 0.5);
    end

    if fit_success(k) && ~isempty(fitted_curve_world{k})
        curve = fitted_curve_world{k};
        if is_closed_layer(k)
            curve_color = BSPLINE_CURVE_COLOR;
        else
            curve_color = OPEN_CURVE_COLOR;
        end
        plot3(ax0, curve(:,1), curve(:,2), curve(:,3), ...
            '-', 'Color', curve_color, 'LineWidth', CURVE_LINE_WIDTH);
    end
end

lgd0 = legend(ax0, {'残腔表面','参考轴线','定向包围盒','第一个切片平面','其余切片平面','拟合轮廓'}, ...
    'Location', 'bestoutside');
set_legend_fonts(lgd0, FONT_CN, FONT_SIZE);
camlight(ax0, 'headlight');
lighting(ax0, 'gouraud');

for k = 1:num_slices
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

    nCand = size(candidate_points_world{k}, 1);
    nRep = size(bin_rep_points_world{k}, 1);
    nFit = size(fitted_curve_world{k}, 1);
    shape_str = ternary_str(is_closed_layer(k), '闭合层', '开口层');
    title(axk, sprintf('第 %d 层：%s，覆盖率=%.3f，候选点=%d，代表点=%d，拟合点=%d', ...
        k, shape_str, angle_coverage(k), nCand, nRep, nFit), ...
        'FontName', FONT_CN, 'FontSize', FONT_SIZE);

    trisurf(F, V_mesh(:,1), V_mesh(:,2), V_mesh(:,3), ...
        'Parent', axk, ...
        'FaceColor', SURFACE_COLOR, ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.08);

    plot3(axk, [contour_result.axis_display_p0(1), contour_result.axis_display_p1(1)], ...
              [contour_result.axis_display_p0(2), contour_result.axis_display_p1(2)], ...
              [contour_result.axis_display_p0(3), contour_result.axis_display_p1(3)], ...
              '-', 'Color', REFERENCE_AXIS_COLOR, 'LineWidth', 2.3);

    Pk = contour_result.slice_planes_world{k};
    if k == 1
        fill3(axk, Pk(:,1), Pk(:,2), Pk(:,3), FIRST_SLICE_PLANE_COLOR, ...
            'FaceAlpha', 0.18, ...
            'EdgeColor', FIRST_SLICE_PLANE_COLOR, ...
            'LineWidth', 1.1);
    else
        fill3(axk, Pk(:,1), Pk(:,2), Pk(:,3), SLICE_PLANE_COLOR, ...
            'FaceAlpha', 0.14, ...
            'EdgeColor', SLICE_PLANE_COLOR, ...
            'LineWidth', 0.9);
    end

    if SHOW_CANDIDATE_POINTS && ~isempty(candidate_points_world{k})
        scatter3(axk, candidate_points_world{k}(:,1), candidate_points_world{k}(:,2), candidate_points_world{k}(:,3), ...
            CANDIDATE_POINT_SIZE, CANDIDATE_POINT_COLOR, 'filled');
    end

    if SHOW_BIN_REP_POINTS && ~isempty(bin_rep_points_world{k})
        scatter3(axk, bin_rep_points_world{k}(:,1), bin_rep_points_world{k}(:,2), bin_rep_points_world{k}(:,3), ...
            REP_POINT_SIZE, REP_POINT_COLOR, 'filled');
    end

    if ~is_closed_layer(k) && SHOW_OPEN_ORDERED_POINTS && ~isempty(open_ordered_points_world{k})
        scatter3(axk, open_ordered_points_world{k}(:,1), open_ordered_points_world{k}(:,2), open_ordered_points_world{k}(:,3), ...
            ORDERED_POINT_SIZE, ORDERED_POINT_COLOR, 'filled');
    end

    if fit_success(k) && ~isempty(fitted_curve_world{k})
        curve = fitted_curve_world{k};
        if is_closed_layer(k)
            curve_color = BSPLINE_CURVE_COLOR;
        else
            curve_color = OPEN_CURVE_COLOR;
        end
        plot3(axk, curve(:,1), curve(:,2), curve(:,3), ...
            '-', 'Color', curve_color, 'LineWidth', CURVE_LINE_WIDTH);
    end

    lgdk = legend(axk, {'残腔表面','参考轴线','当前层切片平面','原始候选点','分箱代表点','开口层有序点','拟合轮廓'}, ...
        'Location', 'bestoutside');
    set_legend_fonts(lgdk, FONT_CN, FONT_SIZE);

    camlight(axk, 'headlight');
    lighting(axk, 'gouraud');
end

%% -------------------------
% 6) 独立弹窗可视化（单独 figure）
%% -------------------------
fig_popup = figure('Color', 'w', ...
    'Name', 'B样条拟合结果独立窗口', ...
    'Position', [120 80 1350 900]);

% 设置新窗口的默认字体：英文/数字统一采用 Times New Roman，字号18
set(fig_popup, 'DefaultAxesFontName', POPUP_FONT_EN, ...
    'DefaultAxesFontSize', POPUP_FONT_SIZE, ...
    'DefaultTextFontName', POPUP_FONT_EN, ...
    'DefaultTextFontSize', POPUP_FONT_SIZE);

ax_popup = axes(fig_popup);
hold(ax_popup, 'on');
axis(ax_popup, 'equal');
grid(ax_popup, 'on');
view(ax_popup, 3);
set_axis_fonts(ax_popup, POPUP_FONT_EN, POPUP_FONT_SIZE);

xlabel(ax_popup, 'X(mm)', 'FontName', POPUP_FONT_EN, 'FontSize', POPUP_FONT_SIZE);
ylabel(ax_popup, 'Y(mm)', 'FontName', POPUP_FONT_EN, 'FontSize', POPUP_FONT_SIZE);
zlabel(ax_popup, 'Z(mm)', 'FontName', POPUP_FONT_EN, 'FontSize', POPUP_FONT_SIZE);
title(ax_popup, '所有层拟合轮廓独立总览', 'FontName', POPUP_FONT_CN, 'FontSize', POPUP_FONT_SIZE);

trisurf(F, V_mesh(:,1), V_mesh(:,2), V_mesh(:,3), ...
    'Parent', ax_popup, ...
    'FaceColor', SURFACE_COLOR, ...
    'EdgeColor', 'none', ...
    'FaceAlpha', 0.12);

plot3(ax_popup, [contour_result.axis_display_p0(1), contour_result.axis_display_p1(1)], ...
                [contour_result.axis_display_p0(2), contour_result.axis_display_p1(2)], ...
                [contour_result.axis_display_p0(3), contour_result.axis_display_p1(3)], ...
      '-', 'Color', REFERENCE_AXIS_COLOR, 'LineWidth', 3.0);

for i = 1:size(E,1)
    p1 = C(E(i,1), :);
    p2 = C(E(i,2), :);
    plot3(ax_popup, [p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
        '-', 'Color', BBOX_COLOR, 'LineWidth', 1.0);
end

for k = 1:num_slices
    Pk = contour_result.slice_planes_world{k};
    if k == 1
        fill3(ax_popup, Pk(:,1), Pk(:,2), Pk(:,3), FIRST_SLICE_PLANE_COLOR, ...
            'FaceAlpha', 0.08, ...
            'EdgeColor', FIRST_SLICE_PLANE_COLOR, ...
            'LineWidth', 0.8);
    else
        fill3(ax_popup, Pk(:,1), Pk(:,2), Pk(:,3), SLICE_PLANE_COLOR, ...
            'FaceAlpha', 0.05, ...
            'EdgeColor', SLICE_PLANE_COLOR, ...
            'LineWidth', 0.5);
    end

    if fit_success(k) && ~isempty(fitted_curve_world{k})
        curve = fitted_curve_world{k};
        if is_closed_layer(k)
            curve_color = BSPLINE_CURVE_COLOR;
        else
            curve_color = OPEN_CURVE_COLOR;
        end
        plot3(ax_popup, curve(:,1), curve(:,2), curve(:,3), ...
            '-', 'Color', curve_color, 'LineWidth', CURVE_LINE_WIDTH);
    end
end

lgd_popup = legend(ax_popup, {'残腔表面','参考轴线','定向包围盒','第一个切片平面','其余切片平面','拟合轮廓'}, ...
    'Location', 'bestoutside');
set_legend_fonts(lgd_popup, POPUP_FONT_CN, POPUP_FONT_SIZE);

camlight(ax_popup, 'headlight');
lighting(ax_popup, 'gouraud');

fprintf('B 样条拟合完成。\n');

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
% 角度覆盖率估计
%% =========================================================
function coverage = estimate_angle_coverage(angle_list_deg, angle_step_deg)
    if isempty(angle_list_deg)
        coverage = 0;
        return;
    end

    nBins = max(round(360 / angle_step_deg), 1);
    angle_list_deg = mod(angle_list_deg, 360);
    bin_id = floor(angle_list_deg / angle_step_deg) + 1;
    bin_id(bin_id > nBins) = nBins;

    occ = false(nBins, 1);
    occ(bin_id) = true;
    coverage = sum(occ) / nBins;
end

%% =========================================================
% 角度分箱代表点：所有点参与，每箱求均值
%% =========================================================
function [uv_rep, rep_counts, rep_angles] = angular_bin_representatives(uv_sorted, theta_sorted, bin_deg, min_count, is_closed)
    if isempty(uv_sorted)
        uv_rep = zeros(0,2);
        rep_counts = [];
        rep_angles = [];
        return;
    end

    if is_closed
        edges = 0:bin_deg:360;
        nBins = numel(edges) - 1;
        uv_rep = zeros(0,2);
        rep_counts = [];
        rep_angles = [];

        for b = 1:nBins
            t0 = edges(b);
            t1 = edges(b+1);

            if t1 < 360
                mask = (theta_sorted >= t0) & (theta_sorted < t1);
            else
                mask = (theta_sorted >= t0) & (theta_sorted <= t1);
            end

            idx = find(mask);
            if numel(idx) < min_count
                continue;
            end

            uv_bin = uv_sorted(idx, :);
            uv_rep(end+1, :) = mean(uv_bin, 1); %#ok<AGROW>
            rep_counts(end+1, 1) = numel(idx); %#ok<AGROW>
            rep_angles(end+1, 1) = t0 + 0.5*bin_deg; %#ok<AGROW>
        end

    else
        tmin = min(theta_sorted);
        tmax = max(theta_sorted);
        if tmax - tmin < 1e-12
            uv_rep = mean(uv_sorted, 1);
            rep_counts = size(uv_sorted, 1);
            rep_angles = mean(theta_sorted);
            return;
        end

        edges = tmin:bin_deg:tmax;
        if edges(end) < tmax
            edges = [edges, tmax];
        end
        if numel(edges) < 2
            edges = [tmin, tmax];
        end

        nBins = numel(edges) - 1;
        uv_rep = zeros(0,2);
        rep_counts = [];
        rep_angles = [];

        for b = 1:nBins
            t0 = edges(b);
            t1 = edges(b+1);

            if b < nBins
                mask = (theta_sorted >= t0) & (theta_sorted < t1);
            else
                mask = (theta_sorted >= t0) & (theta_sorted <= t1);
            end

            idx = find(mask);
            if numel(idx) < min_count
                continue;
            end

            uv_bin = uv_sorted(idx, :);
            uv_rep(end+1, :) = mean(uv_bin, 1); %#ok<AGROW>
            rep_counts(end+1, 1) = numel(idx); %#ok<AGROW>
            rep_angles(end+1, 1) = mean(theta_sorted(idx)); %#ok<AGROW>
        end
    end
end

%% =========================================================
% 环形移动平均（闭合层）
%% =========================================================
function uv_sm = circular_moving_average(uv, win)
    if size(uv,1) < 3 || win <= 1
        uv_sm = uv;
        return;
    end

    win = max(round(win), 1);
    if mod(win,2) == 0
        win = win + 1;
    end
    halfw = floor(win/2);
    n = size(uv,1);

    if n <= 2*halfw
        uv_sm = uv;
        return;
    end

    uv_ext = [uv(end-halfw+1:end,:); uv; uv(1:halfw,:)];
    uv_sm = zeros(size(uv));

    for i = 1:n
        seg = uv_ext(i:i+2*halfw, :);
        uv_sm(i,:) = mean(seg, 1);
    end
end

%% =========================================================
% 开放移动平均（开口层）
%% =========================================================
function uv_sm = open_moving_average(uv, win)
    if size(uv,1) < 3 || win <= 1
        uv_sm = uv;
        return;
    end

    win = max(round(win), 1);
    if mod(win,2) == 0
        win = win + 1;
    end

    uv_sm = uv;
    uv_sm(:,1) = smoothdata(uv(:,1), 'movmean', win);
    uv_sm(:,2) = smoothdata(uv(:,2), 'movmean', win);
end

%% =========================================================
% 去掉连续重复点
%% =========================================================
function P2 = remove_consecutive_duplicate_points(P)
    if isempty(P) || size(P,1) < 2
        P2 = P;
        return;
    end

    keep = true(size(P,1), 1);
    for i = 2:size(P,1)
        if norm(P(i,:) - P(i-1,:)) < 1e-12
            keep(i) = false;
        end
    end
    P2 = P(keep, :);
end

%% =========================================================
% 回退闭合 spline 方案
%% =========================================================
function uv_fit = closed_spline_fallback(uv_closed, n_out)
    d = sqrt(sum(diff(uv_closed, 1, 1).^2, 2));
    s = [0; cumsum(d)];
    if s(end) < eps
        uv_fit = repmat(uv_closed(1,:), n_out, 1);
        return;
    end
    s = s / s(end);

    s_query = linspace(0, 1, n_out);
    u_fit = interp1(s, uv_closed(:,1), s_query, 'spline');
    v_fit = interp1(s, uv_closed(:,2), s_query, 'spline');
    uv_fit = [u_fit(:), v_fit(:)];
end

%% =========================================================
% 开放 spline 方案
%% =========================================================
function uv_fit = open_spline_fallback(uv_open, n_out)
    d = sqrt(sum(diff(uv_open, 1, 1).^2, 2));
    s = [0; cumsum(d)];
    if s(end) < eps
        uv_fit = repmat(uv_open(1,:), n_out, 1);
        return;
    end
    s = s / s(end);

    s_query = linspace(0, 1, n_out);
    u_fit = interp1(s, uv_open(:,1), s_query, 'spline');
    v_fit = interp1(s, uv_open(:,2), s_query, 'spline');
    uv_fit = [u_fit(:), v_fit(:)];
end

%% =========================================================
% 计算离散曲率（二维点列）
%% =========================================================
function curv = compute_discrete_curvature(P)
    n = size(P,1);
    curv = zeros(n,1);

    if n < 3
        return;
    end

    for i = 2:n-1
        p0 = P(i-1,:);
        p1 = P(i,:);
        p2 = P(i+1,:);

        v1 = p1 - p0;
        v2 = p2 - p1;

        L1 = norm(v1);
        L2 = norm(v2);
        if L1 < 1e-12 || L2 < 1e-12
            curv(i) = 0;
            continue;
        end

        c = dot(v1, v2) / (L1 * L2);
        c = max(min(c, 1), -1);
        dtheta = acos(c);
        ds = 0.5 * (L1 + L2);

        if ds < 1e-12
            curv(i) = 0;
        else
            curv(i) = dtheta / ds;
        end
    end

    if n >= 3
        curv(1) = curv(2);
        curv(end) = curv(end-1);
    end
end

%% =========================================================
% 对开口层首尾端进行曲率清理
%% =========================================================
function [P_out, curv_out] = trim_open_curve_by_end_curvature( ...
    P_in, curv_in, end_check_count, peak_ratio, recover_ratio, max_trim_count, min_keep_count)

    P_out = P_in;
    curv_out = curv_in;

    n = size(P_in,1);
    if n < 10
        return;
    end

    end_check_count = min(end_check_count, floor(n/3));
    max_trim_count = min(max_trim_count, floor(n/3));

    mid_start = end_check_count + 1;
    mid_end = n - end_check_count;
    if mid_start >= mid_end
        return;
    end

    curv_mid = curv_in(mid_start:mid_end);
    base_curv = median(curv_mid);

    if base_curv < 1e-12
        base_curv = mean(curv_mid) + 1e-12;
    end

    peak_th = peak_ratio * base_curv;
    recover_th = recover_ratio * base_curv;

    trim_start = 0;
    curv_head = curv_in(1:end_check_count);
    idx_peak_head = find(curv_head > peak_th, 1, 'first');
    if ~isempty(idx_peak_head)
        for i = idx_peak_head:end_check_count
            if curv_in(i) <= recover_th
                trim_start = i - 1;
                break;
            end
        end
        if trim_start == 0
            trim_start = min(end_check_count, max_trim_count);
        end
    end
    trim_start = min(trim_start, max_trim_count);

    trim_end = 0;
    curv_tail = curv_in(n-end_check_count+1:n);
    idx_peak_tail_local = find(curv_tail > peak_th, 1, 'last');
    if ~isempty(idx_peak_tail_local)
        idx_peak_tail = n - end_check_count + idx_peak_tail_local;
        for i = idx_peak_tail:-1:(n-end_check_count+1)
            if curv_in(i) <= recover_th
                trim_end = n - i;
                break;
            end
        end
        if trim_end == 0
            trim_end = min(end_check_count, max_trim_count);
        end
    end
    trim_end = min(trim_end, max_trim_count);

    keep_start = 1 + trim_start;
    keep_end = n - trim_end;

    if keep_end - keep_start + 1 < min_keep_count
        return;
    end

    P_out = P_in(keep_start:keep_end, :);
    curv_out = curv_in(keep_start:keep_end);
end

%% =========================================================
% 设置坐标轴字体
%% =========================================================
function set_axis_fonts(ax, font_en, font_size)
    set(ax, 'FontName', font_en, 'FontSize', font_size);
end

%% =========================================================
% 设置图例字体
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
