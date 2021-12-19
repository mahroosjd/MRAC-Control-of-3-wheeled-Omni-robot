%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------MRAC CONTROL OF A 3-WHEELED ROBOT--------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-----------------NO NOISE-----------------------

function xdot = project_nonoise(t, x)

%direct MRAC
gammax=20; %needed positive
gammar=20;  %needed positive

A = [-3.1303 0 0; 0 -3.0923 0; 0 0 -5.9479];
B = [-0.4118 0 0.4118; 0.2377 0.4755 0.2377; 4.867 4.867 4.867];
Am = [-2.886 0 0;0 -2.886 0; 0 0 -2.886];
Bm = [0 -0.0289 0.0289;0.0333 -0.0167 -0.0167;0.0344 0.0344 0.0344];
K = [-0.9809 0 0;0 -1.0506 0;0 0 -0.0499];
P=lyap(Am',eye(3));


invB = inv(B);
KXstar = invB*(Am-A);



xdot = zeros(30,1);		% Initialize to zero column vector.

%r=[sin(t);cos(t);0];

%r = [t;t;0];

r = [5;5;0];

%r = [0;t^2;0];

xdot(1:3,1)=Am*x(1:3,1)+Bm*r;% This is xmdot
lambda=1;
kxdot1 = vertcat(x(7:9,1)',x(10:12,1)',x(13:15,1)');
kxdot1 = kxdot1';  %Taking Transpose
krdot1 = vertcat(x(16:18,1)',x(19:21,1)',x(22:24,1)');
krdot1 = krdot1';  %Taking Transpose
u = kxdot1*x(4:6,1)+ krdot1*r; %%Both kx and kr are 3x3 matrices

xdot(4:6,1) = A*x(4:6,1)+B*u;	% This is xdot

e=-x(1:3,1)+x(4:6,1);

kxdot = -gammax*eye(3)*x(4:6,1)*(e')*P*B*sign(lambda);	% This is kx dot.(3x3 matrix)
xdot(7:9,1) = kxdot(1,:);
xdot(10:12,1)= kxdot(2,:);
xdot(13:15,1) = kxdot(3,:);

krdot = -gammar*eye(3)*r*(e')*P*B*sign(lambda); % This is kr dot.(3x3 matrix)
xdot(16:18,1)= krdot(1,:);
xdot(19:21,1) = krdot(2,:);
xdot(22:24,1) = krdot(3,:);

%Trying to get x,y and xm,ym to get trajectories
xdot(25:26,1) = x(1:2,1);   %x and y coordinates according to reference model
xdot(27:28,1) = x(4:5,1);   %x and y coordinates according to actual model
xdot(29,1) = x(3,1);   %Angle/Orientation according to Reference model
xdot(30,1) = x(6,1);   %Angle/Orientation according to actual model
end