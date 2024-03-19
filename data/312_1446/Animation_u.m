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
h1 = animatedline('Color','r','LineWidth',1.5);%,'MaximumNumPoints',100);
% h2 = animatedline('Color','r','LineWidth',1.5);%,'MaximumNumPoints',100);
axis([0,120,-0.75,0.28])
% h1 = animatedline('Color','b','LineWidth',1.5);
% h2 = animatedline('Color','k','LineWidth',1.5);
for k=1:length(barrier_info.t)
% addpoints(h,out.x1.signals.values(k),out.x2.signals.values(k))
addpoints(h,double(barrier_info.t(k)),double(barrier_info.u_1(k)))
drawnow 
addpoints(h1,double(barrier_info.t(k)),double(barrier_info.u_2(k)))
drawnow 
% addpoints(h2,out.h_and_yita.time(k),out.U_3.signals(3).values  (k))
% drawnow 
set(gcf,'Position',[600 400 500 150])
label3_1='$u_{1} $';
label3_2='$u_{2} $';
legend(label3_1,label3_2,'FontSize',12,'Interpreter','latex','Location','southeast');
legend('boxoff')
xlabel('Time $(s)$','Interpreter','latex','FontSize',8,'FontName','Times New Roman')
% MakeGif('u.Gif',k)
end