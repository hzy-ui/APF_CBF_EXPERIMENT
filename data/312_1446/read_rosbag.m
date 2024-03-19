
clear;
close all;
bag = rosbag('2024-03-12-14-46-24.bag');
% sel = select(bag, 'Time','StartTime','double');
% start_time;
% end_i=490;
% time_lim=50;
bagSelection = select(bag, 'Topic', '/barrier_information');
% % 6. 导入指定主题的消息
% bagSelection = select(bag, 'Topic', sel);
messages = readMessages(bagSelection,'DataFormat','struct');
barrier_info = struct('x', [], 'y', []);

% 7. 处理导入的消息数据（示例）
for i = 1:numel(messages)
    % 获取消息数据
    msg = messages{i,1};
    
    barrier_info.t(i) = msg.T;
    barrier_info.x(i) = msg.X;
    barrier_info.y(i) = msg.Y;
    barrier_info.h(i) = msg.H;
    barrier_info.gamma(i) = msg.Gamma;
    barrier_info.b(i) = msg.B;
    barrier_info.u_1(i) = msg.U1;
    barrier_info.u_2(i) = msg.U2;
    
end

figure(1)
% 绘制X和Y坐标图形
plot(barrier_info.x  , barrier_info.y  , 'b'); % 使用蓝色线条

% 设置图形标题和坐标轴标签
title('X-Y Coordinates');
xlabel('X');
ylabel('Y');

% 显示网格线
grid on;
