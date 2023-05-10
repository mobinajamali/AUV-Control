function [xdesire,ydesire,zdesire,rolldesire,pitchdesire,yawdesire] = fcn(t)

% this is the desired x, y, z, roll, pitch and yaw equations for spiral
% motion
w=pi/10;
xdesire=exp(-0.01*t)*cos(w*t);
ydesire=exp(-0.01*t)*sin(w*t);
zdesire=0.25*t;
rolldesire=pi*sin(w*t);
pitchdesire=(pi/3)*cos(w*t);
yawdesire=0.25*t;

% %for complex motion (viviani curve)
% R=1;
% w=pi/10;
% 
%  xdesire=2*R*cos(w*t/2)^2;
%  ydesire=2*R*sin(w*t/2)*cos(w*t/2);
%  zdesire=2*R*sin(w*t/2);
%  
%  rolldesire=(pi/2)*sin(w*t);
%  pitchdesire=(pi/3)*sin(w*t);
%  yawdesire=(pi/2)*sin(w*t);
 

end
