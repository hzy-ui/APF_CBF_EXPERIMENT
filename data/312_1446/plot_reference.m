%run this after run the Omnidirectional_robot_V3 file
load('scenario_1a.mat');
figure(1)
% load('playground.mat','x3','y3','x4','y4');
[x1,y1]=circle(60,85,12);
[x2,y2]=circle(20,80,8);
[x3,y3]=circle(50,50,15);
[x4,y4]=circle(90,90,10);
[x5,y5]=circle(0,100,10);
[x6,y6]=circle(100,0,10);
% [x7,y7]=circle(0,0,10);
plot(x1,y1,'r--','LineWidth',1.5)
hold on
plot(x2,y2,'r--','LineWidth',1.5)
hold on
plot(x3,y3,'r','LineWidth',1.5)
hold on
plot(x4,y4,'k','LineWidth',1.5)
hold on
plot(x5,y5,'k','LineWidth',1.5)
hold on 
plot(x6,y6,'k','LineWidth',1.5)
hold on 
% plot(x7,y7,'k','LineWidth',1.5)
% hold on 
agent1=plot(out.x1.signals.values,out.x2.signals.values  ,'-b','LineWidth',1.5);
grid on
label1_1='$traj(\mbox{\boldmath $x$})$';
% label1_2='$traj(x_2^d)$';
% label1_3='$traj(x_3^d)$'
legend([agent1], {label1_1},'FontSize',12,'Interpreter','latex');
legend('boxoff')
xlim([-10 120]);
ylim([-10 120]);
set(gcf,'Position',[0 0 300 250])

figure(2)
plot(out.h_and_yita.time ,out.h_and_yita.signals(1).values,'b','LineWidth',1.5); %yita
hold on
plot(out.h_and_yita.time ,out.h_and_yita.signals(2).values ,'k','LineWidth',1.5);%h
grid on
label2_1='$\gamma (t)$';
label2_2='$h(\mbox{\boldmath $x$})$';
legend(label2_1,label2_2,'FontSize',12,'Interpreter','latex');
legend('boxoff')
legend('Location','southeast');
xlabel('Time $(s)$','Interpreter','latex','FontSize',8,'FontName','Times New Roman')
ylim([-150 35]);
set(gcf,'Position',[100 100 500 150])

% 
figure(3)
plot(out.U_3.time  ,out.U_3.signals(1).values,'g','LineWidth',1.5);
hold on
plot(out.U_3.time  ,out.U_3.signals(2).values,'r','LineWidth',1.5);
hold on
plot(out.U_3.time  ,out.U_3.signals(3).values,'b','LineWidth',1.5);
grid on
label3_1='$u_{1} $';
label3_2='$u_{2} $';
label3_3='$u_{3} $';
legend(label3_1,label3_2,label3_3,'FontSize',12,'Interpreter','latex');
legend('boxoff')
xlabel('Time $(s)$','Interpreter','latex','FontSize',8,'FontName','Times New Roman')
ylim([-2600 2600]);
set(gcf,'Position',[100 100 500 200])
