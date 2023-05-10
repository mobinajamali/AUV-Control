function [xdot,ydot,zdot,phidot,tetadot,saydot] = fcn(u,v,w,p,q,r,phi,teta,say,t,phi0,teta0,say0)

%this block is to generate the current position of the vehicle

%using if statement for checking if it is the initial position or not
if t==0
    phi=phi0;
    teta=teta0;
    say=say0;
end

%for non initial positions, the transformation matrices will be used to
%transform from body to earth frame the corresponding velocities being
%calculated by integration of the acceleration
J1eta2=[cos(say)*cos(teta) , -sin(say)*cos(phi)+cos(say)*sin(teta)*sin(phi) ,sin(say)*sin(phi)+cos(say)*sin(teta)*cos(phi);sin(say)*cos(teta),cos(say)*cos(phi)+sin(say)*sin(teta)*sin(phi),-cos(say)*sin(phi)+sin(say)*sin(teta)*cos(phi);-sin(teta),cos(teta)*sin(phi),cos(teta)*cos(phi)];
J2eta2=[1,sin(phi)*tan(teta),cos(phi)*tan(teta);0,cos(phi),-sin(phi);0,sin(phi)/cos(teta),cos(phi)/cos(teta)];
Jeta=[ J1eta2 zeros(3,3);zeros(3,3) J2eta2 ];

xdot=Jeta(1,1)*u+Jeta(1,2)*v+Jeta(1,3)*w+Jeta(1,4)*p+Jeta(1,5)*q+Jeta(1,6)*r;
ydot=Jeta(2,1)*u+ Jeta(2,2)*v+ Jeta(2,3)*w+Jeta(2,4)*p+Jeta(2,5)*q+Jeta(2,6)*r;
zdot=Jeta(3,1)*u+Jeta(3,2)*v+Jeta(3,3)*w+ Jeta(3,4)*p+ Jeta(3,5)*q+Jeta(3,6)*r;
phidot=Jeta(4,1)*u+Jeta(4,2)*v+Jeta(4,3)*w+ Jeta(4,4)*p+ Jeta(4,5)*q+Jeta(4,6)*r;
tetadot=Jeta(5,1)*u+Jeta(5,2)*v+Jeta(5,3)*w+ Jeta(5,4)*p+ Jeta(5,5)*q+Jeta(5,6)*r;
saydot=Jeta(6,1)*u+Jeta(6,2)*v+ Jeta(6,3)*w+ Jeta(6,4)*p+Jeta(6,5)*q+Jeta(6,6)*r;


