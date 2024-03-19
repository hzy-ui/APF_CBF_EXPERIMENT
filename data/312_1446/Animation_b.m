clc;
clear;
figure(1)
load('barrier_info_312_1446.mat');
% plot(x3,y3,'r','LineWidth',1.5)
% hold on
% plot(x4,y4,'k','LineWidth',1.5)
% hold on
grid on

h = animatedline('Color','b','LineWidth',1.5);%,'MaximumNumPoints',100);
% h1 = animatedline('Color','b','LineWidth',1.5);%,'MaximumNumPoints',100);
% h2 = animatedline('Color','r','LineWidth',1.5);%,'MaximumNumPoints',100);
axis([0,120,-0.1,0.7])
% h1 = animatedline('Color','b','LineWidth',1.5);
% h2 = animatedline('Color','k','LineWidth',1.5);
for k=1:length(barrier_info.t)
% addpoints(h,out.x1.signals.values(k),out.x2.signals.values(k))
addpoints(h,double(barrier_info.t(k)),double(barrier_info.b(k)))
drawnow 
% addpoints(h1,double(barrier_info.t(k)),double(barrier_info.h(k)))
% drawnow 
% addpoints(h2,out.h_and_yita.time(k),out.U_3.signals(3).values  (k))
% drawnow 
set(gcf,'Position',[600 400 500 150])
% label2_1='$\mathfrak{b}$';
% label2_2='$h(x)$';
% legend(label2_1,label2_2,'FontSize',12,'Interpreter','latex');
% legend('boxoff')
xlabel('Time $(s)$','Interpreter','latex','FontSize',8,'FontName','Times New Roman')
% MakeGif('b.Gif',k)
end