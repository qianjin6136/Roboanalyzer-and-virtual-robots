%% 清理工作区及命令行等
clc,clear,close all;

%% 定义五角星轨迹点

% 计算五角星坐标信息
R = 0.1;
theta = 0:72:359;
x = 0.1 + R * cosd(theta);
y = 0.2 + R * sind(theta);
z = 0.6 * ones(size(x));

% 连接构建五角星路径点
path = [x(1), y(1), z(1);
        x(3), y(3), z(3);
        x(5), y(5), z(5);
        x(2), y(2), z(2);
        x(4), y(4), z(4);
        x(1), y(1), z(1)];

% 设置路径的参数信息
distances = vecnorm(diff(path), 2, 2);
max_speed = 0.1;  % 轨迹中的最大速度 m/s
segment_times = distances / max_speed;
total_points = 1000;  % 总插值点数
traj = [];

% 计算机器人路径信息
for i = 1:size(path,1)-1
    % 计算当前段需要的点数 (按距离比例)
    seg_length = norm(path(i+1,:) - path(i,:));
    num_points = round(total_points * seg_length / sum(distances));   
    % 线性插值
    seg_traj = interp1([0 1], [path(i,:); path(i+1,:)], linspace(0,1,num_points));
    traj = [traj; seg_traj];
end

%% 定义6R机器人
L(1) = Link([0 0.40 0.18 pi/2]);
L(2) = Link([0 0 0.4 0]);
L(3) = Link([0 0 0.12 -pi/2]);
L(4) = Link([0 0.4 0 pi/2]);
L(5) = Link([0 0 0 -pi/2]);
L(6) = Link([0 0 0.12 0]);
R_6= SerialLink(L, 'name', '6R_Robot');

%% 逆运动学求解关节参数信息

% 提取位置和姿态向量
positions = traj(:,1:3);  % xyz
T_list = [];  % 用于保存所有变换矩阵
for i = 1:size(positions,1)
    % 构造变换矩阵：设末端姿态固定为 [0 0 -1]
    pos = positions(i,:);
    % 定义一个固定姿态（z 向下，y 向左）
    R = [1 0 0;
         0 1 0;
         0 0 -1];
    T = rt2tr(R, pos');  % 构造齐次变换矩阵
    T_list(:,:,i) = T;
end

% 设置初始关节角猜测
q0 = [0, 0, 0, 0, 0, 0];
% 使用逆运动学求解
for i = 1:size(T_list,3)
    T = T_list(:,:,i);
    q_all(i,:) = R_6.ikine(T, 'q0', q0);
end

%% 计算并输出运动的csv文件

% 计算总轨迹长度和总时间
diff_traj = diff(traj);
traj_lengths = sqrt(sum(diff_traj.^2, 2)); % 各段轨迹长度
total_length = sum(traj_lengths);
total_time = total_length / max_speed;

% 生成时间向量 (均匀采样)
N = size(traj, 1); % 总轨迹点数
time_vector = linspace(0, total_time, N)';
dt = total_time / (N - 1); % 时间步长

% 关节角度转换 (弧度 -> 角度)
q_deg = q_all * (180/pi);

% 预分配速度加速度矩阵
velocity = zeros(N, 6);
acceleration = zeros(N, 6);

% 计算速度和加速度 (中心差分法)
for j = 1:6 % 对每个关节循环
    % 速度计算
    for k = 1:N
        if k == 1
            % 前向差分 (起始点)
            velocity(k,j) = (q_deg(k+1,j) - q_deg(k,j)) / dt;
        elseif k == N
            % 后向差分 (结束点)
            velocity(k,j) = (q_deg(k,j) - q_deg(k-1,j)) / dt;
        else
            % 中心差分
            velocity(k,j) = (q_deg(k+1,j) - q_deg(k-1,j)) / (2*dt);
        end
    end
    
    % 加速度计算
    for k = 1:N
        if k == 1
            % 前向差分 (起始点)
            acceleration(k,j) = (q_deg(k+2,j) - 2*q_deg(k+1,j) + q_deg(k,j)) / (dt^2);
        elseif k == N
            % 后向差分 (结束点)
            acceleration(k,j) = (q_deg(k,j) - 2*q_deg(k-1,j) + q_deg(k-2,j)) / (dt^2);
        else
            % 中心差分
            acceleration(k,j) = (q_deg(k+1,j) - 2*q_deg(k,j) + q_deg(k-1,j)) / (dt^2);
        end
    end
end

% 创建输出矩阵: [时间, j1_pos, j1_vel, j1_acc, j2_pos, j2_vel, j2_acc, ...]
output_data = zeros(N, 19); % 1列时间 + 6关节×(3个量) = 19列
output_data(:, 1) = time_vector;

% 填充关节数据
for j = 0:5
    col_idx = 2 + j*3;
    output_data(:, col_idx)   = q_deg(:, j+1);     % 位置
    output_data(:, col_idx+1) = velocity(:, j+1);   % 速度
    output_data(:, col_idx+2) = acceleration(:, j+1); % 加速度
end

% 写入CSV文件
filename = 'joint_data.csv';
writematrix(output_data, filename, 'Delimiter', ',');
disp(['数据已成功写入文件: ', filename]);

%% 计算末端轨迹数据

% 1. 位置数据 (直接从轨迹获取)
ee_position = traj; % [x, y, z]

% 2. 计算线速度 (单位: m/s)
ee_velocity = zeros(N, 3);
for k = 1:N
    if k == 1
        ee_velocity(k,:) = (ee_position(k+1,:) - ee_position(k,:)) / dt;
    elseif k == N
        ee_velocity(k,:) = (ee_position(k,:) - ee_position(k-1,:)) / dt;
    else
        ee_velocity(k,:) = (ee_position(k+1,:) - ee_position(k-1,:)) / (2*dt);
    end
end

% 3. 计算线加速度 (单位: m/s²)
ee_acceleration = zeros(N, 3);
for k = 1:N
    if k == 1
        ee_acceleration(k,:) = (ee_position(k+2,:) - 2*ee_position(k+1,:) + ee_position(k,:)) / (dt^2);
    elseif k == N
        ee_acceleration(k,:) = (ee_position(k,:) - 2*ee_position(k-1,:) + ee_position(k-2,:)) / (dt^2);
    else
        ee_acceleration(k,:) = (ee_position(k+1,:) - 2*ee_position(k,:) + ee_position(k-1,:)) / (dt^2);
    end
end

%% 绘制末端轨迹时程曲线
figure('Name', '末端执行器轨迹', 'Position', [100, 100, 1200, 800]);

% 1. 位置曲线
subplot(3,1,1);
plot(time_vector, ee_position(:,1), 'r-', 'LineWidth', 1.5);
hold on;
plot(time_vector, ee_position(:,2), 'g-', 'LineWidth', 1.5);
plot(time_vector, ee_position(:,3), 'b-', 'LineWidth', 1.5);
hold off;
title('末端位置');
xlabel('时间 (s)');
ylabel('位置 (m)');
legend('X', 'Y', 'Z');
grid on;

% 2. 速度曲线
subplot(3,1,2);
plot(time_vector, ee_velocity(:,1), 'r-', 'LineWidth', 1.5);
hold on;
plot(time_vector, ee_velocity(:,2), 'g-', 'LineWidth', 1.5);
plot(time_vector, ee_velocity(:,3), 'b-', 'LineWidth', 1.5);
hold off;
title('末端速度');
xlabel('时间 (s)');
ylabel('速度 (m/s)');
legend('Vx', 'Vy', 'Vz');
grid on;

% 3. 加速度曲线
subplot(3,1,3);
plot(time_vector, ee_acceleration(:,1), 'r-', 'LineWidth', 1.5);
hold on;
plot(time_vector, ee_acceleration(:,2), 'g-', 'LineWidth', 1.5);
plot(time_vector, ee_acceleration(:,3), 'b-', 'LineWidth', 1.5);
hold off;
title('末端加速度');
xlabel('时间 (s)');
ylabel('加速度 (m/s²)');
legend('Ax', 'Ay', 'Az');
grid on;

% 保存图为高分辨率PNG
saveas(gcf, 'end_traj.png');
disp('数据处理完成!');