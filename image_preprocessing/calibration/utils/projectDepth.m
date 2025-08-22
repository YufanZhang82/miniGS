function depthMap = projectDepth(obj_vertices, K, A, imageWidth, imageHeight)
    % 将3D顶点从世界坐标系转换为相机坐标系
    vertices_camera = (A * [obj_vertices'; ones(1, size(obj_vertices, 1))])';
    
    % 提取相机坐标中的深度（Z坐标）
    Z = vertices_camera(:, 3);  % 深度值
    
    % 归一化齐次坐标，获得图像中的2D投影点
    proj_points = (K * vertices_camera(:, 1:3)')';
    proj_points = proj_points(:, 1:2) ./ proj_points(:, 3);  % 转为2D点
    
    % 创建空的深度图并填充
    depthMap = inf(imageHeight, imageWidth);  % 初始化为无穷大表示很远的地方
    for i = 1:size(proj_points, 1)
        x = round(proj_points(i, 1));
        y = round(proj_points(i, 2));
        if x > 0 && x <= imageWidth && y > 0 && y <= imageHeight
            depthMap(y, x) = min(depthMap(y, x), Z(i));  % 保存最小的深度值
        end
    end
    
    % 将inf值替换为0
    depthMap(isinf(depthMap)) = 0;
end

