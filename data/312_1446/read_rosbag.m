
clear;
close all;
bag = rosbag('2024-03-12-14-46-24.bag');
% sel = select(bag, 'Time','StartTime','double');
% start_time;
% end_i=490;
% time_lim=50;
bagSelection = select(bag, 'Topic', '/barrier_information');
% % 6. ����ָ���������Ϣ
% bagSelection = select(bag, 'Topic', sel);
messages = readMessages(bagSelection,'DataFormat','struct');
barrier_info = struct('x', [], 'y', []);

% 7. ���������Ϣ���ݣ�ʾ����
for i = 1:numel(messages)
    % ��ȡ��Ϣ����
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
% ����X��Y����ͼ��
plot(barrier_info.x  , barrier_info.y  , 'b'); % ʹ����ɫ����

% ����ͼ�α�����������ǩ
title('X-Y Coordinates');
xlabel('X');
ylabel('Y');

% ��ʾ������
grid on;
