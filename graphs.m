X1 = 1:1001;
X10 = 1:10:1000;

figure(1);
% subplot(2,3,1);
plot(X1, Error(1,:)*180/pi,'rx',X10, Error(2,X10)*180/pi,'b+', X1, Error(4,:)*180/pi,'k', X10, Error(5,X10)*180/pi,'g',X10, Error(6,X10)*180/pi,'go');
ylabel('Error in \theta (deg)','FontSize',14)
xlabel('Time (ms)','FontSize',14)
legend('\theta_1','\theta_2','\theta_4','\theta_5','\theta_6')
title('Error in joint angles (\theta_i)' ,'FontSize',14)
% subplot(2,3,2);
% figure(2);
% plot(X1, Error(2,:),'r');
% ylabel('Error in \theta_2 (rad)','FontSize',14,'fontweight','b')
% xlabel('Time (sec)','FontSize',14,'fontweight','b')
% % subplot(2,3,3);
figure(3);
plot(X1, Error(3,:),'rx');
ylabel('Error in r_3 (m)','FontSize',14)
xlabel('Time (ms)','FontSize',14)
title('Error in r_3','FontSize',14)
% % subplot(2,3,4);
% figure(4);
% plot(X1, Error(4,:),'k');
% ylabel('Error in \theta_4 (rad)','FontSize',14,'fontweight','b')
% xlabel('Time (sec)','FontSize',14,'fontweight','b')
% % subplot(2,3,5);
% figure(5);
% plot(X1, Error(5,:),'g');
% ylabel('Error in \theta_5 (rad)','FontSize',14,'fontweight','b')
% xlabel('Time (sec)','FontSize',14,'fontweight','b')
% % subplot(2,3,6);
% figure(6);
% plot(X1, Error(6,:),'o');
% ylabel('Error in \theta_6 (rad)','FontSize',14,'fontweight','b')
% xlabel('Time (sec)','FontSize',14,'fontweight','b')

figure(7);
% subplot(2,3,1);
plot(X1, ErrorDot(1,:),'rx',X10, ErrorDot(2,X10),'b+', X1, ErrorDot(4,:),'k', X10, ErrorDot(5,X10),'g',X10, ErrorDot(6,X10),'go');
ylabel('Error in \omega (rad/s)','FontSize',14)
xlabel('Time (ms)','FontSize',14)
legend('\omega_1','\omega_2','\omega_4','\omega_5','\omega_6')
title('Error in joint velocities (\omega_i)' ,'FontSize',14)
% subplot(2,3,2);
% figure(8);
% plot(X10, ErrorDot(2,X10),'rx', X10, ErrorDot(5,X10),'g');
% ylabel('Error in \omega (rad)','FontSize',14,'fontweight','b')
% xlabel('Time (ms)','FontSize',14,'fontweight','b')
% legend('\omega_2','\omega_5')
% subplot(2,3,3);
figure(9);
plot(X1, ErrorDot(3,:),'rx');
ylabel('Error in v_3 (m/s)','FontSize',14)
xlabel('Time (ms)','FontSize',14)
title('Error in v_3', 'FontSize',14)
% subplot(2,3,4);
% figure(10);
% plot(X1, ErrorDot(4,:),'k');
% ylabel('Error in \omega_4 (rad)','FontSize',14,'fontweight','b')
% xlabel('Time (sec)','FontSize',14,'fontweight','b')
% % subplot(2,3,5);
% figure(11);
% plot(X1, ErrorDot(5,:),'g');
% ylabel('Error in \omega_5 (rad)','FontSize',14,'fontweight','b')
% xlabel('Time (sec)','FontSize',14,'fontweight','b')
% % subplot(2,3,6);
% figure(12);
% plot(X1, ErrorDot(6,:),'o');
% ylabel('Error in \omega_6 (rad)','FontSize',14,'fontweight','b')
% xlabel('Time (sec)','FontSize',14,'fontweight','b')

figure(13);
plot(X1,ActualPos(:,1)*180/pi,'rx',X1,ThetaDesired(:,1)*180/pi,'b')
ylabel('\theta (deg)','FontSize',14)
xlabel('Time (ms)','FontSize',14)
legend('Actual','Desired')
title('\theta_1 v/s time' ,'FontSize',14)
axis([0 1000 -90 180])

figure(14);
plot(X1,ActualPos(:,2)*180/pi,'rx',X1,ThetaDesired(:,2)*180/pi,'b')
ylabel('\theta (deg)','FontSize',14)
xlabel('Time (ms)','FontSize',14)
legend('Actual','Desired')
title('\theta_2 v/s time' ,'FontSize',14)

figure(15);
plot(X1,ActualPos(:,3),'rx',X1,ThetaDesired(:,3),'b')
ylabel('r_3 (m)','FontSize',14)
xlabel('Time (ms)','FontSize',14)
legend('Actual','Desired')
title('r_3 v/s time' ,'FontSize',14)

figure(16);
plot(X1,ActualPos(:,4)*180/pi,'rx',X1,ThetaDesired(:,4)*180/pi,'b')
ylabel('\theta (deg)','FontSize',14)
xlabel('Time (ms)','FontSize',14)
legend('Actual','Desired')
title('\theta_4 v/s time' ,'FontSize',14)
axis([0 1000 -30 30])

figure(17);
plot(X1,ActualPos(:,5)*180/pi,'rx',X1,ThetaDesired(:,5)*180/pi,'b')
ylabel('\theta (deg)','FontSize',14)
xlabel('Time (ms)','FontSize',14)
legend('Actual','Desired')
title('\theta_5 v/s time' ,'FontSize',14)

figure(18);
plot(X1,ActualPos(:,6)*180/pi,'rx',X1,ThetaDesired(:,6)*180/pi,'b')
ylabel('\theta (deg)','FontSize',14)
xlabel('Time (ms)','FontSize',14)
legend('Actual','Desired')
title('\theta_6 v/s time' ,'FontSize',14)

% figure(19);
% plot(X1, TauFF(1,:),X1, TauFB(1,:));
% ylabel('Torque (N-m)','FontSize',14)
% xlabel('Time (ms)','FontSize',14)
% legend('Feedforward','Feedback')
% title('Torque on joint 1 v/s time' ,'FontSize',14)
% 
% figure(20);
% plot(X1, TauFF(2,:),X1, TauFB(2,:));
% ylabel('Torque (N-m)','FontSize',14)
% xlabel('Time (ms)','FontSize',14)
% legend('Feedforward','Feedback')
% title('Torque on joint 2 v/s time' ,'FontSize',14)
% 
% figure(21);
% plot(X1, TauFF(3,:),X1, TauFB(3,:));
% ylabel('Torque (N-m)','FontSize',14)
% xlabel('Time (ms)','FontSize',14)
% legend('Feedforward','Feedback')
% title('Torque on joint 3 v/s time' ,'FontSize',14)
% 
% figure(22);
% plot(X1, TauFF(4,:),X1, TauFB(4,:));
% ylabel('Torque (N-m)','FontSize',14)
% xlabel('Time (ms)','FontSize',14)
% legend('Feedforward','Feedback')
% title('Torque on joint 4 v/s time' ,'FontSize',14)
% 
% figure(23);
% plot(X1, TauFF(5,:),X1, TauFB(5,:));
% ylabel('Torque (N-m)','FontSize',14)
% xlabel('Time (ms)','FontSize',14)
% legend('Feedforward','Feedback')
% title('Torque on joint 5 v/s time' ,'FontSize',14)
% 
% figure(24);
% plot(X1, TauFF(6,:),X1, TauFB(6,:));
% ylabel('Torque (N-m)','FontSize',14)
% xlabel('Time (ms)','FontSize',14)
% legend('Feedforward','Feedback')
% title('Torque on joint 6 v/s time' ,'FontSize',14)