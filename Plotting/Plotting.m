
%desire trajectory parameters
xdesire = out.xdesire;
ydesire = out.ydesire;
zdesire = out.zdesire;
rolldesire = out.rolldesire;
pitchdesire = out.pitchdesire;
yawdesire = out.yawdesire;
M=out.M;
cond_M=out.Cond;

%simulated trajectory
surge = out.surge;
sway = out.sway;
heave = out.heave;
roll = out.roll;
pitch = out.pitch;
yaw = out.yaw;

%simulated velocities
u = out.u;
v = out.v;
w = out.w;
p = out.p;
q = out.q;
r = out.r;


%simulated forces
%F=out.F;
f1=out.f1;
f2=out.f2;
f3=out.f3;

%simulated powers
P1=out.P1;
P2=out.P2;
P3=out.P3;

%simulated teta angles
Teta1=out.Teta1;
Teta2=out.Teta2;
Teta3=out.Teta3;


start_point = [xdesire.data(1), ydesire.data(1), zdesire.data(1)];
end_point = [xdesire.data(end), ydesire.data(end), zdesire.data(end)];

%plotting all the corresponding figures
figure(1)
plot3(xdesire.data,ydesire.data,zdesire.data);
hold on;
plot3(surge.data,sway.data,heave.data,'color','red');
plot3(start_point(1), start_point(2), start_point(3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot3(end_point(1), end_point(2), end_point(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
legend('Desired','Simulation','start','end');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
title('Viviani Trajectory');
hold off;

figure(2)
plot(xdesire);
hold on;
plot(surge,'color','red');
legend('Desired','Simulation');
xlabel('t(s)');
ylabel('x(m)');
title('Comparison between the desired and simulated surge motion');
hold off;

figure(3)
plot(ydesire);
hold on;
plot(sway,'color','red');
legend('Desired', 'simulation');
xlabel('t(s)');
ylabel('y(m)');
title('Comparison between the desired and simulated sway motion');
hold off;

figure(4)
plot(zdesire);
hold on;
plot(heave,'color','red');
legend('Desired','Simulation');
xlabel('t(s)');
ylabel('z(m)');
title('Comparison between the desired and simulated heave motion');
hold off;
% 
figure(5)
plot(rolldesire);
hold on;
plot(roll,'color','red');
legend('Desired','Simulation');
xlabel('t(s)');
ylabel('phi(rad)');
title('Comparison between the desired and simulated roll rate');
hold off;
% 
figure(6)
plot(pitchdesire);
hold on;
plot(pitch,'color','red');
legend('Desired','Simulation')
xlabel('t(s)');
ylabel('theta(rad)');
title('Comparison between the desired and simulated pitch rate');
hold off;

figure(7)
plot(yawdesire);
hold on;
plot(yaw,'color','red');
legend('Desired','Simulation');
xlabel('t(s)');
ylabel('say(rad)');
title('Comparison between the desired and simulated yaw rate');
hold off;

figure(8)
plot(u); 
xlabel('t(s)');
ylabel('u(m/s)');
title('Surge velocity');

figure(9)
plot(v); 
xlabel('t(s)');
ylabel('v(m/s)');
title('Sway velocity');

figure(10)
plot(w); 
xlabel('t(s)');
ylabel('w(m/s)');
title('heave velocity');

figure(11)
plot(p); 
xlabel('t(s)');
ylabel('p(m/s)');
title('roll velocity');

figure(12)
plot(q); 
xlabel('t(s)');
ylabel('q(m/s)');
title('pitch velocity');

figure(13)
plot(r);
xlabel('t(s)');
ylabel('r(m/s)');
title('yaw velocity');
% 
figure(14)
plot(f1);
hold on
plot(f2,'color','red');
hold on; 
plot(f3,'color','green');
legend('f1','f2','f3');
xlabel('t(s)');
ylabel('thrust force(N)');
title('Thruster forces');
hold off;

% 
% theta1=cos(Teta1);
% theta2=cos(Teta2);
% theta3=cos(Teta3);

figure(16)
plot(Teta1);
hold off
plot(Teta2,'color','red');
hold on; 
plot(Teta3,'color','green');
legend('Theta1','Theta2','Theta3');
xlabel('t(s)');
ylabel('Thruster Angle (rad)');
title('Thruster Tilting Angles');
hold off;

