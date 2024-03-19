%run this after run the Omnidirectional_robot_V3 file
figure(1)
% load('playground.mat','x3','y3','x4','y4');
[x1,y1]=circle(60,80,12);
[x2,y2]=circle(10,70,5);
[x3,y3]=circle(50,50,15);
[x4,y4]=circle(90,90,10);
[x5,y5]=circle(0,100,10);
[x6,y6]=circle(100,0,10);
[x7,y7]=circle(0,0,10);
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
plot(x7,y7,'k','LineWidth',1.5)
hold on 
agent1=plot(out.x1.signals.values,out.x2.signals.values  ,'-r','LineWidth',1.5);
hold on
agent2=plot(out.x1_agent2.signals.values,out.x2_agent2.signals.values  ,'-b','LineWidth',1.5);
hold on
agent3=plot(out.x1_agent3.signals.values,out.x2_agent3.signals.values  ,'-m','LineWidth',1.5);
grid on

label1_1='$traj(x_1^d)$';
label1_2='$traj(x_2^d)$';
label1_3='$traj(x_3^d)$'
legend([agent1 agent2 agent3], {label1_1,label1_2,label1_3},'FontSize',12,'Interpreter','latex');
legend('boxoff')
xlim([-10 120]);
ylim([-10 120]);

figure(2)
plot(out.h_and_yita.time,out.h_and_yita.signals(1).values,'b','LineWidth',1.5); %yita
hold on
plot(out.h_and_yita.time,out.h_and_yita.signals(2).values,'k','LineWidth',1.5);%h
grid on
label2_1='$\gamma (t)$';
label2_2='$h(x)$';
legend(label2_1,label2_2,'FontSize',12,'Interpreter','latex');
legend('boxoff')
xlabel('Time $(s)$','Interpreter','latex','FontSize',8,'FontName','Times New Roman')
ylim([-150 30]);
% 
figure(3)
plot(out.U_3.time,out.U_3.signals(1).values,'g','LineWidth',1.5);
hold on
plot(out.U_3.time,out.U_3.signals(2).values,'r','LineWidth',1.5);
hold on
plot(out.U_3.time,out.U_3.signals(3).values,'b','LineWidth',1.5);
grid on
label3_1='$u_{1} $';
label3_2='$u_{2} $';
label3_3='$u_{3} $';
legend(label3_1,label3_2,label3_3,'FontSize',12,'Interpreter','latex');
legend('boxoff')
xlabel('Time $(s)$','Interpreter','latex','FontSize',8,'FontName','Times New Roman')
ylim([-2600 2600]);
figure(4) %distance
plot(out.distance1_2.time,out.distance1_2.signals.values ,'g','LineWidth',1.5);
hold on
plot(out.distance1_3.time,out.distance1_3.signals.values,'r','LineWidth',1.5);
hold on
plot(out.distance2_3.time,out.distance2_3.signals.values,'b','LineWidth',1.5);
grid on
label3_1='$dist(x_1^d,x_2^d ) $';
label3_2='$dist(x_1^d,x_3^d )$';
label3_3='$dist(x_2^d,x_3^d ) $';
legend(label3_1,label3_2,label3_3,'FontSize',12,'Interpreter','latex');
legend('boxoff')
xlabel('Time $(s)$','Interpreter','latex','FontSize',8,'FontName','Times New Roman')