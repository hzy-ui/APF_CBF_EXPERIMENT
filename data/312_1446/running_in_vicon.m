%run this file after run the 'Omnidirectional_robot_obstacle_avoidance'or
%'Multi_Omnidirectional_robot_V3'
figure(1)
load('barrier_info_312_1446.mat');
% load('playground.mat','x3','y3','x4','y4','x5','y5');
[x1,y1]=circle(2, 3.20, 0.5);
[x2,y2]=circle(-2, 3.17,0.5);
[x3,y3]=circle(1,-3.5,0.5);
[x4,y4]=circle(0.12, 3.61,0.36);
[x5,y5]=circle(0.27, -0.56,0.56);
% [x6,y6]=circle(100,0,10);
% [x7,y7]=circle(0,0,10);
plot(x1,y1,'k','LineWidth',1.5)
hold on
plot(x2,y2,'k','LineWidth',1.5)
hold on
plot(x3,y3,'k','LineWidth',1.5)
hold on
plot(x4,y4,'r','LineWidth',1.5)
hold on
plot(x5,y5,'r','LineWidth',1.5)
hold on 
% plot(x6,y6,'k','LineWidth',1.5)
% hold on 
% plot(x7,y7,'k','LineWidth',1.5)
grid on 
% ylim([-150 35]);
% xlim([0,60]);
% set(gcf,'Position',[100 100 600 200])
h = animatedline('Color','b','LineWidth',1.5);
% h1 = animatedline('Color','r','LineWidth',1.5);
% h2 = animatedline('Color','b','LineWidth',1.5);
% h3 = animatedline('Color','k','LineWidth',1.5);
% h4 = animatedline('Color','k','LineWidth',1.5);
axis([-3,3,-5,5])
for k=1:length(barrier_info.t)    
addpoints(h,barrier_info.x(k),barrier_info.y(k))
drawnow
set(gcf,'Position',[600 200 250 333])
% addpoints(h1,barrier_info.t(k),barrier_info.gamma(k))
% drawnow
% addpoints(h2,barrier_info.t(k),barrier_info.h(k))
% drawnow
% label2_1='$\gamma (t)$';
% label2_2='$h(\mbox{\boldmath $x$})$';
% legend(label2_1,label2_2,'FontSize',12,'Interpreter','latex');
% legend('boxoff')
% xlabel('Time $(s)$','Interpreter','latex','FontSize',8,'FontName','Times New Roman')
% label1_1='$traj(\mbox{\boldmath $x$})$';
% legend([h],label1_1,'FontSize',10,'Interpreter','latex','Location','southwest');
% MakeGif('traj.Gif',k)
 end