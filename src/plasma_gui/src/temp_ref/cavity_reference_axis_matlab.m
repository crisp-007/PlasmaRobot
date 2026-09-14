
function result = cavity_reference_axis_matlab()
% =========================================================
% 残腔参考轴线确定（MATLAB版）
% 参考 zhouxian3.py 的总体流程改写：
% STL网格 -> 表面体素化 -> 3D填充 -> 距离场 ->
% 中心候选点 -> 开口边界与深度方向 -> 分层中心点 ->
% 平滑弯曲中心线 -> PCA拟合直参考轴线 -> 可视化
%
% 说明：
% 1) 输入为 STL 文件
% 2) 可视化采用一个带标签页的窗口：
%    - 标签1：弯曲中心点 / 弯曲中心线
%    - 标签2：最终直参考轴线
% 3) 代码尽量贴近你给的 Python 版处理逻辑
%
% 依赖：
% - Image Processing Toolbox（imfill, bwdist, imdilate）
% =========================================================

%% =========================
% 用户参数区：直接修改这里即可
%% =========================
% 动态获取脚本所在目录，构建文件路径
script_dir = fileparts(mfilename('fullpath'));
INPUT_MESH_PATH = fullfile(script_dir, '..', 'cq_data', 'Mesh1_cj.stl');     % 输入 STL 文件
VOXEL_SIZE = 0.001;                 % 体素尺寸，单位：mm（若 STL 为 m，请改成 0.001）
MEDIAL_PERCENTILE = 75.0;         % 距离场高值百分位
N_SECTIONS = 60;                  % 分层数量
SECTION_HALF_WIDTH = 0.002;         % 截面半宽，单位与 STL 一致；[] 表示自动计算
SMOOTH_POINTS_NUM = 120;          % 平滑后中心线点数
SMOOTH_WINDOW = 7;                % 中心线平滑窗口（奇数更稳）
EXCLUDE_OPENING_DISTANCE = 0.003;   % 开口附近剔除距离；[] 表示不剔除
EXCLUDE_OPENING_LAYERS = [];      % 额外剔除前若干层；[] 表示不剔除
SHOW_MEDIAL_POINTS = true;        % 可视化是否显示中心候选点

SURFACE_COLOR = [0.82 0.82 0.82];
CURVED_POINT_COLOR = [1.0 0.0 0.0];
CURVED_LINE_COLOR  = [0.0 0.25 1.0];
STRAIGHT_AXIS_COLOR = [0.85 0.0 0.85];
DEPTH_AXIS_COLOR = [1.0 0.55 0.0];
OPENING_BOUNDARY_COLOR = [1.0 0.6 0.0];

%% =========================
% 主流程
%% =========================
fprintf('============================================================\n');
fprintf('残腔参考轴线确定（MATLAB版）\n');
fprintf('输入网格: %s\n', INPUT_MESH_PATH);
fprintf('体素尺寸: %.6f\n', VOXEL_SIZE);
fprintf('中心候选百分位: %.2f\n', MEDIAL_PERCENTILE);
fprintf('截面数量: %d\n', N_SECTIONS);
fprintf('截面半宽: %s\n', value_to_str(SECTION_HALF_WIDTH));
fprintf('开口距离剔除: %s\n', value_to_str(EXCLUDE_OPENING_DISTANCE));
fprintf('开口层数剔除: %s\n', value_to_str(EXCLUDE_OPENING_LAYERS));
fprintf('============================================================\n');

[F, V] = load_stl_mesh(INPUT_MESH_PATH);

fprintf('[1/8] 表面体素化...\n');
[surface_occ, origin, voxel_size] = voxelize_mesh_by_triangle_sampling(F, V, VOXEL_SIZE);

fprintf('[2/8] 填充内部体素...\n');
solid = imfill(surface_occ, 'holes');

fprintf('[3/8] 计算欧氏距离场...\n');
dist = bwdist(~solid);

fprintf('[4/8] 提取中心候选点...\n');
medial_mask = extract_medial_candidates(dist, solid, MEDIAL_PERCENTILE);
medial_idx = find(medial_mask);
[ix, iy, iz] = ind2sub(size(medial_mask), medial_idx);
medial_grid = [ix, iy, iz];
medial_points = voxel_indices_to_world(medial_grid, origin, voxel_size);
fprintf('中心候选点数量: %d\n', size(medial_points, 1));

fprintf('[5/8] 提取开口边界并拟合深度方向...\n');
opening_points = largest_boundary_loop_points(F, V);
[opening_center, depth_axis] = opening_based_depth_axis(V, opening_points);
fprintf('开口边界点数量: %d\n', size(opening_points, 1));
fprintf('开口中心: [%.6f, %.6f, %.6f]\n', opening_center(1), opening_center(2), opening_center(3));
fprintf('深度方向: [%.6f, %.6f, %.6f]\n', depth_axis(1), depth_axis(2), depth_axis(3));

fprintf('[6/8] 剔除开口附近不稳定中心候选点...\n');
medial_points_filtered = filter_medial_points_near_opening( ...
    medial_points, opening_center, depth_axis, EXCLUDE_OPENING_DISTANCE);

fprintf('[7/8] 沿深度方向提取弯曲中心线...\n');
coarse_centerline = build_centerline_from_sections_along_axis( ...
    medial_points_filtered, opening_center, depth_axis, ...
    N_SECTIONS, SECTION_HALF_WIDTH, EXCLUDE_OPENING_LAYERS);

curved_centerline_pts = smooth_centerline_points( ...
    coarse_centerline, SMOOTH_POINTS_NUM, SMOOTH_WINDOW);

curved_length = centerline_length(curved_centerline_pts);
fprintf('弯曲中心线点数: %d\n', size(curved_centerline_pts, 1));
fprintf('弯曲中心线长度: %.6f\n', curved_length);

fprintf('[8/8] 对弯曲中心线进行直轴拟合...\n');
[line_center, line_dir] = fit_straight_line(curved_centerline_pts);
[straight_axis_p0, straight_axis_p1] = line_segment_from_points(curved_centerline_pts, line_center, line_dir);
straight_length = norm(straight_axis_p1 - straight_axis_p0);
fprintf('直轴中心点: [%.6f, %.6f, %.6f]\n', line_center(1), line_center(2), line_center(3));
fprintf('直轴方向: [%.6f, %.6f, %.6f]\n', line_dir(1), line_dir(2), line_dir(3));
fprintf('直轴长度: %.6f\n', straight_length);

result = struct();
result.F = F;
result.V = V;
result.opening_points = opening_points;
result.opening_center = opening_center;
result.depth_axis = depth_axis;
result.medial_points = medial_points_filtered;
result.coarse_centerline = coarse_centerline;
result.curved_centerline_pts = curved_centerline_pts;
result.line_center = line_center;
result.line_dir = line_dir;
result.straight_axis_p0 = straight_axis_p0;
result.straight_axis_p1 = straight_axis_p1;
result.voxel_origin = origin;
result.voxel_size = voxel_size;

visualize_with_tabs(result, ...
    SHOW_MEDIAL_POINTS, SURFACE_COLOR, CURVED_POINT_COLOR, CURVED_LINE_COLOR, ...
    STRAIGHT_AXIS_COLOR, DEPTH_AXIS_COLOR, OPENING_BOUNDARY_COLOR);

fprintf('处理完成。\n');
end

function [F, V] = load_stl_mesh(mesh_path)
    TR = stlread(mesh_path);
    if isa(TR, 'triangulation')
        F = TR.ConnectivityList;
        V = TR.Points;
    elseif isstruct(TR)
        if isfield(TR, 'ConnectivityList')
            F = TR.ConnectivityList;
            V = TR.Points;
        elseif isfield(TR, 'faces')
            F = TR.faces;
            V = TR.vertices;
        else
            error('stlread 返回格式不支持，请检查 MATLAB 版本。');
        end
    else
        error('无法解析 STL 文件。');
    end
    if isempty(F) || isempty(V)
        error('读取 STL 失败，网格为空。');
    end
end

function [occ_grid, origin, voxel_size] = voxelize_mesh_by_triangle_sampling(F, V, voxel_size)
    vmin = min(V, [], 1);
    vmax = max(V, [], 1);
    pad = 3 * voxel_size;
    origin = vmin - pad;
    dims = ceil((vmax - vmin + 2*pad) ./ voxel_size) + 1;
    dims = max(dims, [8 8 8]);
    occ_grid = false(dims(1), dims(2), dims(3));

    for i = 1:size(F,1)
        p1 = V(F(i,1), :);
        p2 = V(F(i,2), :);
        p3 = V(F(i,3), :);

        area = triangle_area(p1, p2, p3);
        if area < eps
            continue;
        end

        n_samp = max(30, ceil(area / (0.35 * voxel_size^2)));
        P = sample_points_on_triangle(p1, p2, p3, n_samp);

        idx = floor((P - origin) ./ voxel_size) + 1;
        valid = idx(:,1) >= 1 & idx(:,1) <= dims(1) & ...
                idx(:,2) >= 1 & idx(:,2) <= dims(2) & ...
                idx(:,3) >= 1 & idx(:,3) <= dims(3);
        idx = idx(valid, :);
        lin = sub2ind(dims, idx(:,1), idx(:,2), idx(:,3));
        occ_grid(lin) = true;
    end

    se = strel('sphere', 1);
    occ_grid = imdilate(occ_grid, se);
end

function area = triangle_area(p1, p2, p3)
    area = 0.5 * norm(cross(p2 - p1, p3 - p1));
end

function P = sample_points_on_triangle(p1, p2, p3, n)
    r1 = rand(n,1);
    r2 = rand(n,1);
    swap_mask = (r1 + r2) > 1;
    r1(swap_mask) = 1 - r1(swap_mask);
    r2(swap_mask) = 1 - r2(swap_mask);
    P = p1 + r1 .* (p2 - p1) + r2 .* (p3 - p1);
    P = [P; p1; p2; p3];
end

function medial_mask = extract_medial_candidates(dist, solid, percentile)
    vals = dist(solid);
    if isempty(vals)
        error('未找到内部体素，请检查 STL 是否近似闭合。');
    end
    thresh = prctile(vals, percentile);
    medial_mask = (dist >= thresh) & solid;
end

function pts = voxel_indices_to_world(indices, origin, voxel_size)
    pts = origin + (indices - 0.5) * voxel_size;
end

function opening_points = largest_boundary_loop_points(F, V)
    edges = [F(:,[1 2]); F(:,[2 3]); F(:,[3 1])];
    edges = sort(edges, 2);
    [uEdges, ~, ic] = unique(edges, 'rows');
    cnt = accumarray(ic, 1);
    boundary_edges = uEdges(cnt == 1, :);

    if isempty(boundary_edges)
        error('未检测到开口边界。请检查网格是否存在开口。');
    end

    nV = size(V, 1);
    G = graph(boundary_edges(:,1), boundary_edges(:,2), [], nV);
    comp = conncomp(G);

    bvs = unique(boundary_edges(:));
    comp_ids = comp(bvs);
    uComp = unique(comp_ids);

    best_comp = uComp(1);
    best_num = 0;
    for i = 1:numel(uComp)
        cur = uComp(i);
        cur_nodes = bvs(comp_ids == cur);
        if numel(cur_nodes) > best_num
            best_num = numel(cur_nodes);
            best_comp = cur;
        end
    end

    loop_nodes = bvs(comp_ids == best_comp);
    opening_points = V(loop_nodes, :);
end

function [centroid, normal] = fit_plane_normal(points)
    centroid = mean(points, 1);
    X = points - centroid;
    C = (X' * X) / max(size(points,1)-1, 1);
    [vecs, vals] = eig(C);
    [~, idx] = min(diag(vals));
    normal = vecs(:, idx)';
    normal = normal / norm(normal);
end

function [opening_center, normal] = opening_based_depth_axis(V, opening_points)
    [opening_center, normal] = fit_plane_normal(opening_points);
    mesh_center = mean(V, 1);
    vec_to_inside = mesh_center - opening_center;
    if dot(normal, vec_to_inside) < 0
        normal = -normal;
    end
end

function filtered = filter_medial_points_near_opening(medial_points, axis_origin, axis_direction, exclude_distance)
    if isempty(exclude_distance) || exclude_distance <= 0
        filtered = medial_points;
        return;
    end
    axis_direction = axis_direction / norm(axis_direction);
    t = (medial_points - axis_origin) * axis_direction(:);
    mask = t >= exclude_distance;
    filtered = medial_points(mask, :);
    if size(filtered, 1) < 10
        warning('按开口距离剔除后剩余点过少，已返回原始中心候选点。');
        filtered = medial_points;
    end
end

function centerline = build_centerline_from_sections_along_axis( ...
    medial_points, axis_origin, axis_direction, n_sections, section_half_width, exclude_opening_layers)

    axis_direction = axis_direction / norm(axis_direction);
    t = (medial_points - axis_origin) * axis_direction(:);
    t_min = min(t);
    t_max = max(t);
    t_range = t_max - t_min;
    section_centers = linspace(t_min, t_max, n_sections);

    if isempty(section_half_width)
        section_half_width = t_range / (2.5 * n_sections);
    end

    if ~isempty(exclude_opening_layers) && exclude_opening_layers > 0
        exclude_opening_layers = min(exclude_opening_layers, numel(section_centers)-1);
        section_centers = section_centers((exclude_opening_layers+1):end);
    end

    fprintf('[DEBUG] t_min=%.6f, t_max=%.6f, t_range=%.6f\n', t_min, t_max, t_range);
    fprintf('[DEBUG] section_half_width=%.6f\n', section_half_width);

    centerline = zeros(0, 3);
    for i = 1:numel(section_centers)
        tc = section_centers(i);
        mask = abs(t - tc) <= section_half_width;
        pts = medial_points(mask, :);
        if size(pts, 1) < 5
            continue;
        end
        centerline(end+1, :) = mean(pts, 1); %#ok<AGROW>
    end

    if size(centerline, 1) < 4
        error('提取到的截面中心点过少。可尝试调整 MEDIAL_PERCENTILE / N_SECTIONS / SECTION_HALF_WIDTH。');
    end
end

function smoothed = smooth_centerline_points(points, n_out, smooth_window)
    diffs = diff(points, 1, 1);
    seglen = sqrt(sum(diffs.^2, 2));
    u = [0; cumsum(seglen)];
    if u(end) < eps
        smoothed = points;
        return;
    end
    u = u / u(end);

    pts = points;
    for d = 1:3
        pts(:,d) = smoothdata(pts(:,d), 'movmean', smooth_window);
    end

    u_new = linspace(0, 1, n_out);
    smoothed = zeros(n_out, 3);
    for d = 1:3
        smoothed(:,d) = interp1(u, pts(:,d), u_new, 'pchip');
    end
end

function L = centerline_length(points)
    if size(points,1) < 2
        L = 0;
        return;
    end
    L = sum(sqrt(sum(diff(points,1,1).^2, 2)));
end

function [center, direction] = fit_straight_line(points)
    center = mean(points, 1);
    X = points - center;
    C = (X' * X) / max(size(points,1)-1, 1);
    [vecs, vals] = eig(C);
    [~, idx] = max(diag(vals));
    direction = vecs(:, idx)';
    direction = direction / norm(direction);
end

function [p0, p1] = line_segment_from_points(points, center, direction)
    direction = direction / norm(direction);
    t = (points - center) * direction(:);
    t_min = min(t);
    t_max = max(t);
    p0 = center + t_min * direction;
    p1 = center + t_max * direction;
end

function visualize_with_tabs(result, ...
    show_medial_points, surface_color, curved_point_color, curved_line_color, ...
    straight_axis_color, depth_axis_color, opening_boundary_color)

    % 创建统一窗口，带标签页
    fig = figure('Color','w','Name','残腔参考轴线可视化','Position', [100 80 1400 900]);
    tg = uitabgroup(fig);

    % 标签页1：弯曲中心点
    tab1 = uitab(tg, 'Title', '弯曲中心点');
    ax1 = axes(tab1);
    hold(ax1, 'on'); axis(ax1, 'equal'); grid(ax1, 'on'); view(ax1, 3);
    title(ax1, '各层中心点组成的弯曲中心线');
    xlabel(ax1, 'X'); ylabel(ax1, 'Y'); zlabel(ax1, 'Z');

    trisurf(result.F, result.V(:,1), result.V(:,2), result.V(:,3), ...
        'Parent', ax1, 'FaceColor', surface_color, 'EdgeColor', 'none', 'FaceAlpha', 0.35);

    if show_medial_points
        scatter3(ax1, result.medial_points(:,1), result.medial_points(:,2), result.medial_points(:,3), ...
            4, [0.0 0.75 0.0], 'filled', 'MarkerFaceAlpha', 0.18, 'MarkerEdgeAlpha', 0.18);
    end

    scatter3(ax1, result.opening_points(:,1), result.opening_points(:,2), result.opening_points(:,3), ...
        14, opening_boundary_color, 'filled');

    plot3(ax1, result.coarse_centerline(:,1), result.coarse_centerline(:,2), result.coarse_centerline(:,3), ...
        '-o', 'Color', [0.90 0.35 0.20], 'LineWidth', 1.2, 'MarkerSize', 4, ...
        'MarkerFaceColor', [0.90 0.35 0.20]);

    scatter3(ax1, result.curved_centerline_pts(:,1), result.curved_centerline_pts(:,2), result.curved_centerline_pts(:,3), ...
        18, curved_point_color, 'filled');
    plot3(ax1, result.curved_centerline_pts(:,1), result.curved_centerline_pts(:,2), result.curved_centerline_pts(:,3), ...
        '-', 'Color', curved_line_color, 'LineWidth', 2.6);

    bbox_min = min(result.V, [], 1);
    bbox_max = max(result.V, [], 1);
    diag_len = norm(bbox_max - bbox_min);
    p_depth_end = result.opening_center + 0.8 * diag_len * result.depth_axis;
    plot3(ax1, [result.opening_center(1), p_depth_end(1)], ...
              [result.opening_center(2), p_depth_end(2)], ...
              [result.opening_center(3), p_depth_end(3)], ...
              '-', 'Color', depth_axis_color, 'LineWidth', 2.2);

    legend(ax1, {'表面网格','中心候选点','开口边界','各层中心点','平滑中心线点','平滑中心线','深度方向'}, ...
        'Location', 'bestoutside');
    camlight(ax1, 'headlight'); lighting(ax1, 'gouraud');

    % 标签页2：直参考轴线
    tab2 = uitab(tg, 'Title', '直参考轴线');
    ax2 = axes(tab2);
    hold(ax2, 'on'); axis(ax2, 'equal'); grid(ax2, 'on'); view(ax2, 3);
    title(ax2, 'PCA 拟合后的直参考轴线');
    xlabel(ax2, 'X'); ylabel(ax2, 'Y'); zlabel(ax2, 'Z');

    trisurf(result.F, result.V(:,1), result.V(:,2), result.V(:,3), ...
        'Parent', ax2, 'FaceColor', surface_color, 'EdgeColor', 'none', 'FaceAlpha', 0.28);

    scatter3(ax2, result.curved_centerline_pts(:,1), result.curved_centerline_pts(:,2), result.curved_centerline_pts(:,3), ...
        16, curved_point_color, 'filled');
    plot3(ax2, [result.straight_axis_p0(1), result.straight_axis_p1(1)], ...
              [result.straight_axis_p0(2), result.straight_axis_p1(2)], ...
              [result.straight_axis_p0(3), result.straight_axis_p1(3)], ...
              '-', 'Color', straight_axis_color, 'LineWidth', 4.0);

    scatter3(ax2, result.line_center(1), result.line_center(2), result.line_center(3), ...
        60, straight_axis_color, 'filled', 'MarkerEdgeColor', 'k');

    legend(ax2, {'表面网格','弯曲中心线点','直参考轴线','PCA中心点'}, ...
        'Location', 'bestoutside');
    camlight(ax2, 'headlight'); lighting(ax2, 'gouraud');
end

function s = value_to_str(v)
    if isempty(v)
        s = '[]';
    elseif isnumeric(v) && isscalar(v)
        s = num2str(v);
    else
        s = mat2str(v);
    end
end
