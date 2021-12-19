close all
clear all
clc

tspan = [0 150];% Time span.
Ln = 2;

x0 = [0 0 0 0 0 0 -0.2966 -0.4338 +0.6290 0 +0.8675 -0.6288 +0.2966 -0.4388 +0.6290 -0.063 0.0773 0.0071 0.133 -0.0773 -0.0773 -0.063 0.0071 0.0773 0 0 0 0 0 0];			
%x0 = [0 0 0 0 0 0 22 0 0 0 22 0 0 0 0 1 0 0 0 1 0 0 0 0];
%x0 = [0 0 0 0 0 0 0.2 0.4 -0.6 0 -0.8 0.6 -0.2 0.4 -0.6 -0.05 0.07 0.006 0.1 -0.07 -0.07 -0.05 0.006 0.07];

%%     DIRECT MRAC- NO NOISE

[t, x] = ode45(@project_nonoise , tspan, x0'); %third order system

figure;

plot(t,x(:,1),'b','LineWidth',Ln);hold on
plot(t,x(:,4),'r','LineWidth',Ln)
title('Linear Velocities x1 and xm1 - NO NOISE'); legend('Reference model','Actual');
figure
plot(t,x(:,2),'b','LineWidth',Ln);hold on
plot(t,x(:,5),'r','LineWidth',Ln)
title('Linear Velocitites x2 and xm2 - NO NOISE'); legend('Reference model','Actual');
figure
plot(t,x(:,3),'b','LineWidth',Ln);hold on
plot(t,x(:,6),'r','LineWidth',Ln)
title('Angular Velocities x3 and xm3 - NO NOISE'); legend('Reference model','Actual');

figure;
subplot(3,1,1)
plot(t,x(:,1)-x(:,4),'g','LineWidth',Ln)
title('Error xm1-x1  NO NOISE')
subplot(3,1,2)
plot(t,x(:,2)-x(:,5),'g','LineWidth',Ln)
title('Error xm2-x2  NO NOISE')
subplot(3,1,3)
plot(t,x(:,3)-x(:,6),'g','LineWidth',Ln)
title('Error xm3-x3  NO NOISE')

figure;
subplot(3,1,1)
plot(t,x(:,7),'g','LineWidth',Ln)
title('kx(1,1)- NO NOISE')
subplot(3,1,2)
plot(t,x(:,8),'g','LineWidth',Ln)
title('kx(1,2)- NO NOISE')
subplot(3,1,3)
plot(t,x(:,9),'g','LineWidth',Ln)
title('kx(1,3)- NO NOISE')

figure;
subplot(3,1,1)
plot(t,x(:,10),'g','LineWidth',Ln)
title('kx(2,1)- NO NOISE')
subplot(3,1,2)
plot(t,x(:,11),'g','LineWidth',Ln)
title('kx(2,2)- NO NOISE')
subplot(3,1,3)
plot(t,x(:,12),'g','LineWidth',Ln)
title('kx(2,3)- NO NOISE')

figure;
subplot(3,1,1)
plot(t,x(:,13),'g','LineWidth',Ln)
title('kx(3,1)- NO NOISE')
subplot(3,1,2)
plot(t,x(:,14),'g','LineWidth',Ln)
title('kx(3,2)- NO NOISE')
subplot(3,1,3)
plot(t,x(:,15),'g','LineWidth',Ln)
title('kx(3,3)- NO NOISE')

%%%Gain Matrix Kr%%%%%
figure;
subplot(3,1,1)
plot(t,x(:,16),'g','LineWidth',Ln)
title('kr(1,1)- NO NOISE')
subplot(3,1,2)
plot(t,x(:,17),'g','LineWidth',Ln)
title('kr(1,2)- NO NOISE')
subplot(3,1,3)
plot(t,x(:,18),'g','LineWidth',Ln)
title('kr(1,3)- NO NOISE')

figure;
subplot(3,1,1)
plot(t,x(:,19),'g','LineWidth',Ln)
title('kr(2,1)- NO NOISE')
subplot(3,1,2)
plot(t,x(:,20),'g','LineWidth',Ln)
title('kr(2,2)- NO NOISE')
subplot(3,1,3)
plot(t,x(:,21),'g','LineWidth',Ln)
title('kr(2,3)- NO NOISE')

figure;
subplot(3,1,1)
plot(t,x(:,22),'g','LineWidth',Ln)
title('kr(3,1)- NO NOISE')
subplot(3,1,2)
plot(t,x(:,23),'g','LineWidth',Ln)
title('kr(3,2)- NO NOISE')
subplot(3,1,3)
plot(t,x(:,24),'g','LineWidth',Ln)
title('kr(3,3)- NO NOISE')

%%  ----------ROBOT TRAJECTORY--------------
%%Plotting the trajectory using x,y coordinates and theta (By solving
%%differential equations vx = dx/dt, vy = dy/dt, w = dtheta/dt) 
figure;
subplot(2,1,1)
plot(x(:,25).*cos(x(:,29)),x(:,26).*sin(x(:,29)),'b','LineWidth',2)
title('Trajectory of Reference Model')
xlabel('x coordinate') 
ylabel('y coordinate')
subplot(2,1,2)
plot(x(:,27).*cos(x(:,30)),x(:,28).*sin(x(:,30)),'b','LineWidth',2)
title('Trajectory followed by actual System') 
xlabel('x coordinate') 
ylabel('y coordinate')


%%     DIRECT MRAC- WITH NOISE (ZERO MEAN WHITE NOISE, ST. DEVIATION 0.1)
x0 = [0 0 0 0 0 0 -0.2966 -0.4338 +0.6290 0 +0.8675 -0.6288 +0.2966 -0.4388 +0.6290 -0.063 0.0773 0.0071 0.133 -0.0773 -0.0773 -0.063 0.0071 0.0773];	
[t, x] = ode45(@project_noise , tspan, x0'); %third order system

figure;

plot(t,x(:,1),'b','LineWidth',Ln);hold on
plot(t,x(:,4),'r','LineWidth',Ln)
title('Linear Velocities x1 and xm1 - WITH NOISE ST. DEV 0.1'); legend('Reference model','Actual');
figure
plot(t,x(:,2),'b','LineWidth',Ln);hold on
plot(t,x(:,5),'r','LineWidth',Ln)
title('Linear Velocitites x2 and xm2 - WITH NOISE ST. DEV 0.1'); legend('Reference model','Actual');
figure
plot(t,x(:,3),'b','LineWidth',Ln);hold on
plot(t,x(:,6),'r','LineWidth',Ln)
title('Angular Velocities x3 and xm3 - WITH NOISE ST. DEV 0.1'); legend('Reference model','Actual');

figure;
subplot(3,1,1)
plot(t,x(:,1)-x(:,4),'g','LineWidth',Ln)
title('Error xm1-x1  WITH NOISE ST. DEV 0.1')
subplot(3,1,2)
plot(t,x(:,2)-x(:,5),'g','LineWidth',Ln)
title('Error xm2-x2  WITH NOISE ST. DEV 0.1')
subplot(3,1,3)
plot(t,x(:,3)-x(:,6),'g','LineWidth',Ln)
title('Error xm3-x3  WITH NOISE ST. DEV 0.1')

figure;
subplot(3,1,1)
plot(t,x(:,7),'g','LineWidth',Ln)
title('kx(1,1)- WITH NOISE ST. DEV 0.1')
subplot(3,1,2)
plot(t,x(:,8),'g','LineWidth',Ln)
title('kx(1,2)- WITH NOISE ST. DEV 0.1')
subplot(3,1,3)
plot(t,x(:,9),'g','LineWidth',Ln)
title('kx(1,3)- WITH NOISE ST. DEV 0.1')

figure;
subplot(3,1,1)
plot(t,x(:,10),'g','LineWidth',Ln)
title('kx(2,1)- WITH NOISE ST. DEV 0.1')
subplot(3,1,2)
plot(t,x(:,11),'g','LineWidth',Ln)
title('kx(2,2)- WITH NOISE ST. DEV 0.1')
subplot(3,1,3)
plot(t,x(:,12),'g','LineWidth',Ln)
title('kx(2,3)- WITH NOISE ST. DEV 0.1')

figure;
subplot(3,1,1)
plot(t,x(:,13),'g','LineWidth',Ln)
title('kx(3,1)- WITH NOISE ST. DEV 0.1')
subplot(3,1,2)
plot(t,x(:,14),'g','LineWidth',Ln)
title('kx(3,2)- WITH NOISE ST. DEV 0.1')
subplot(3,1,3)
plot(t,x(:,15),'g','LineWidth',Ln)
title('kx(3,3)- WITH NOISE ST. DEV 0.1')

%%%Gain Matrix Kr%%%%%
figure;
subplot(3,1,1)
plot(t,x(:,16),'g','LineWidth',Ln)
title('kr(1,1)- WITH NOISE ST. DEV 0.1')
subplot(3,1,2)
plot(t,x(:,17),'g','LineWidth',Ln)
title('kr(1,2)- WITH NOISE ST. DEV 0.1')
subplot(3,1,3)
plot(t,x(:,18),'g','LineWidth',Ln)
title('kr(1,3)- WITH NOISE ST. DEV 0.1')

figure;
subplot(3,1,1)
plot(t,x(:,19),'g','LineWidth',Ln)
title('kr(2,1)- WITH NOISE ST. DEV 0.1')
subplot(3,1,2)
plot(t,x(:,20),'g','LineWidth',Ln)
title('kr(2,2)- WITH NOISE ST. DEV 0.1')
subplot(3,1,3)
plot(t,x(:,21),'g','LineWidth',Ln)
title('kr(2,3)- WITH NOISE ST. DEV 0.1')

figure;
subplot(3,1,1)
plot(t,x(:,22),'g','LineWidth',Ln)
title('kr(3,1)- WITH NOISE ST. DEV 0.1')
subplot(3,1,2)
plot(t,x(:,23),'g','LineWidth',Ln)
title('kr(3,2)- WITH NOISE ST. DEV 0.1')
subplot(3,1,3)
plot(t,x(:,24),'g','LineWidth',Ln)
title('kr(3,3)- WITH NOISE ST. DEV 0.1')


%%     DIRECT MRAC- SIGMA MODIFICATION
x0 = [0 0 0 0 0 0 -0.2966 -0.4338 +0.6290 0 +0.8675 -0.6288 +0.2966 -0.4388 +0.6290 -0.063 0.0773 0.0071 0.133 -0.0773 -0.0773 -0.063 0.0071 0.0773];	
[t, x] = ode45(@project_sigma , tspan, x0'); %third order system

figure;

plot(t,x(:,1),'b','LineWidth',Ln);hold on
plot(t,x(:,4),'r','LineWidth',Ln)
title('Linear Velocities x1 and xm1 - SIGMA MODIFICATION'); legend('Reference model','Actual');
figure
plot(t,x(:,2),'b','LineWidth',Ln);hold on
plot(t,x(:,5),'r','LineWidth',Ln)
title('Linear Velocitites x2 and xm2 - SIGMA MODIFICATION'); legend('Reference model','Actual');
figure
plot(t,x(:,3),'b','LineWidth',Ln);hold on
plot(t,x(:,6),'r','LineWidth',Ln)
title('Angular Velocities x3 and xm3 - SIGMA MODIFICATION'); legend('Reference model','Actual');

figure;
subplot(3,1,1)
plot(t,x(:,1)-x(:,4),'g','LineWidth',Ln)
title('Error xm1-x1  SIGMA MODIFICATION')
subplot(3,1,2)
plot(t,x(:,2)-x(:,5),'g','LineWidth',Ln)
title('Error xm2-x2  SIGMA MODIFICATION')
subplot(3,1,3)
plot(t,x(:,3)-x(:,6),'g','LineWidth',Ln)
title('Error xm3-x3  SIGMA MODIFICATION')

figure;
subplot(3,1,1)
plot(t,x(:,7),'g','LineWidth',Ln)
title('kx(1,1)- SIGMA MODIFICATION')
subplot(3,1,2)
plot(t,x(:,8),'g','LineWidth',Ln)
title('kx(1,2)- SIGMA MODIFICATION')
subplot(3,1,3)
plot(t,x(:,9),'g','LineWidth',Ln)
title('kx(1,3)- SIGMA MODIFICATION')

figure;
subplot(3,1,1)
plot(t,x(:,10),'g','LineWidth',Ln)
title('kx(2,1)- SIGMA MODIFICATION')
subplot(3,1,2)
plot(t,x(:,11),'g','LineWidth',Ln)
title('kx(2,2)- SIGMA MODIFICATION')
subplot(3,1,3)
plot(t,x(:,12),'g','LineWidth',Ln)
title('kx(2,3)- SIGMA MODIFICATION')

figure;
subplot(3,1,1)
plot(t,x(:,13),'g','LineWidth',Ln)
title('kx(3,1)- SIGMA MODIFICATION')
subplot(3,1,2)
plot(t,x(:,14),'g','LineWidth',Ln)
title('kx(3,2)- SIGMA MODIFICATION')
subplot(3,1,3)
plot(t,x(:,15),'g','LineWidth',Ln)
title('kx(3,3)- SIGMA MODIFICATION')

%%%Gain Matrix Kr%%%%%
figure;
subplot(3,1,1)
plot(t,x(:,16),'g','LineWidth',Ln)
title('kr(1,1)- SIGMA MODIFICATION')
subplot(3,1,2)
plot(t,x(:,17),'g','LineWidth',Ln)
title('kr(1,2)- SIGMA MODIFICATION')
subplot(3,1,3)
plot(t,x(:,18),'g','LineWidth',Ln)
title('kr(1,3)- SIGMA MODIFICATION')

figure;
subplot(3,1,1)
plot(t,x(:,19),'g','LineWidth',Ln)
title('kr(2,1)- SIGMA MODIFICATION')
subplot(3,1,2)
plot(t,x(:,20),'g','LineWidth',Ln)
title('kr(2,2)- SIGMA MODIFICATION')
subplot(3,1,3)
plot(t,x(:,21),'g','LineWidth',Ln)
title('kr(2,3)- SIGMA MODIFICATION')

figure;
subplot(3,1,1)
plot(t,x(:,22),'g','LineWidth',Ln)
title('kr(3,1)- SIGMA MODIFICATION')
subplot(3,1,2)
plot(t,x(:,23),'g','LineWidth',Ln)
title('kr(3,2)- SIGMA MODIFICATION')
subplot(3,1,3)
plot(t,x(:,24),'g','LineWidth',Ln)
title('kr(3,3)- SIGMA MODIFICATION')


%%     DIRECT MRAC- EPSILON MODIFICATION
x0 = [0 0 0 0 0 0 -0.2966 -0.4338 +0.6290 0 +0.8675 -0.6288 +0.2966 -0.4388 +0.6290 -0.063 0.0773 0.0071 0.133 -0.0773 -0.0773 -0.063 0.0071 0.0773];	
[t, x] = ode45(@project_epsilon , tspan, x0'); %third order system

figure;

plot(t,x(:,1),'b','LineWidth',Ln);hold on
plot(t,x(:,4),'r','LineWidth',Ln)
title('Linear Velocities x1 and xm1 - EPSILON MODIFICATION'); legend('Reference model','Actual');
figure
plot(t,x(:,2),'b','LineWidth',Ln);hold on
plot(t,x(:,5),'r','LineWidth',Ln)
title('Linear Velocitites x2 and xm2 - EPSILON MODIFICATION'); legend('Reference model','Actual');
figure
plot(t,x(:,3),'b','LineWidth',Ln);hold on
plot(t,x(:,6),'r','LineWidth',Ln)
title('Angular Velocities x3 and xm3 - EPSILON MODIFICATION'); legend('Reference model','Actual');

figure;
subplot(3,1,1)
plot(t,x(:,1)-x(:,4),'g','LineWidth',Ln)
title('Error xm1-x1  EPSILON MODIFICATION')
subplot(3,1,2)
plot(t,x(:,2)-x(:,5),'g','LineWidth',Ln)
title('Error xm2-x2  EPSILON MODIFICATION')
subplot(3,1,3)
plot(t,x(:,3)-x(:,6),'g','LineWidth',Ln)
title('Error xm3-x3  EPSILON MODIFICATION')

figure;
subplot(3,1,1)
plot(t,x(:,7),'g','LineWidth',Ln)
title('kx(1,1)- EPSILON MODIFICATION')
subplot(3,1,2)
plot(t,x(:,8),'g','LineWidth',Ln)
title('kx(1,2)- EPSILON MODIFICATION')
subplot(3,1,3)
plot(t,x(:,9),'g','LineWidth',Ln)
title('kx(1,3)- EPSILON MODIFICATION')

figure;
subplot(3,1,1)
plot(t,x(:,10),'g','LineWidth',Ln)
title('kx(2,1)- EPSILON MODIFICATION')
subplot(3,1,2)
plot(t,x(:,11),'g','LineWidth',Ln)
title('kx(2,2)- EPSILON MODIFICATION')
subplot(3,1,3)
plot(t,x(:,12),'g','LineWidth',Ln)
title('kx(2,3)- EPSILON MODIFICATION')

figure;
subplot(3,1,1)
plot(t,x(:,13),'g','LineWidth',Ln)
title('kx(3,1)- EPSILON MODIFICATION')
subplot(3,1,2)
plot(t,x(:,14),'g','LineWidth',Ln)
title('kx(3,2)- EPSILON MODIFICATION')
subplot(3,1,3)
plot(t,x(:,15),'g','LineWidth',Ln)
title('kx(3,3)- EPSILON MODIFICATION')

%%%Gain Matrix Kr%%%%%
figure;
subplot(3,1,1)
plot(t,x(:,16),'g','LineWidth',Ln)
title('kr(1,1)- EPSILON MODIFICATION')
subplot(3,1,2)
plot(t,x(:,17),'g','LineWidth',Ln)
title('kr(1,2)- EPSILON MODIFICATION')
subplot(3,1,3)
plot(t,x(:,18),'g','LineWidth',Ln)
title('kr(1,3)- EPSILON MODIFICATION')

figure;
subplot(3,1,1)
plot(t,x(:,19),'g','LineWidth',Ln)
title('kr(2,1)- EPSILON MODIFICATION')
subplot(3,1,2)
plot(t,x(:,20),'g','LineWidth',Ln)
title('kr(2,2)- EPSILON MODIFICATION')
subplot(3,1,3)
plot(t,x(:,21),'g','LineWidth',Ln)
title('kr(2,3)- EPSILON MODIFICATION')

figure;
subplot(3,1,1)
plot(t,x(:,22),'g','LineWidth',Ln)
title('kr(3,1)- EPSILON MODIFICATION')
subplot(3,1,2)
plot(t,x(:,23),'g','LineWidth',Ln)
title('kr(3,2)- EPSILON MODIFICATION')
subplot(3,1,3)
plot(t,x(:,24),'g','LineWidth',Ln)
title('kr(3,3)- EPSILON MODIFICATION')





