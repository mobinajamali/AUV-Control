 function [accelaration,TAVcontrol,F,f1,thruster1teta,f2,thruster2teta,f3,thruster3teta]  = fcn(e,u,v,w,p,q,r,phi,teta,say,phi0,teta0,say0,u0,v0,w0,p0,q0,r0,t,tavdisturbance)

% setting everything to zero as the initial condition of the vehicle
fxthruster1=0;
fxthruster2=0;
fxthruster3=0;

fythruster1=0;
fythruster2=0;
fythruster3=0;

fzthruster1=0;
fzthruster2=0;
fzthruster3=0;

if t==0
    phi=phi0;
    teta=teta0;
    say=say0;
    u=u0;
    v=v0;
    w=w0;
    p=p0;
    q=q0;
    r=r0;
   
end

check=0;


%physical parameters of the vehicle (m)
%w=yl=yr;
width=0.45;
%l=xt+xl=xt+xr;
l=0.885;
m= 12.437;



%%Jacobian aka transformation Matrix for linear velovcity conversion from
%%earth to body frame
J1eta2=[cos(say)*cos(teta) , -sin(say)*cos(phi)+cos(say)*sin(teta)*sin(phi) ,sin(say)*sin(phi)+cos(say)*sin(teta)*cos(phi);sin(say)*cos(teta),cos(say)*cos(phi)+sin(say)*sin(teta)*sin(phi),-cos(say)*sin(phi)+sin(say)*sin(teta)*cos(phi);-sin(teta),cos(teta)*sin(phi),cos(teta)*cos(phi)];
%%Jacobian Matrix for angular velovcity conversionfrom earth to body frame
J2eta2=[1,sin(phi)*tan(teta),cos(phi)*tan(teta);0,cos(phi),-sin(phi);0,sin(phi)/cos(teta),cos(phi)/cos(teta)]
%%combining both of Jacobian matrices to one matrix 
Jeta=[ J1eta2 zeros(3,3);zeros(3,3) J2eta2 ]
%inverse of jeta matrix
Jinv=inv(Jeta)



%other physical parameters of the vehicle (m)
%xg=yr=yl;
xg=0.45;
%yg=xt;
yg=0.75;
zg=0.12;

%vehicle's inertial parameters (kgm^2)
Ixx=6741.44;
Ixy=1906.18;
Ixz=3580.59;
Iyx=1906.18;
Iyy=8454.36;
Iyz=2508.82;
Izx=3580.59;
Izy=2508.82;
Izz=4399.80;

%%hydrodynamic coefficients (to be defined by simulation, probably Ansys),
%%same data is being used as the work of Mr. Kadkhodaei. More work needs to
%%be done to gather data 
Xudot=-39.2
Xvdot=0;
Xwdot=0;
Xpdot=0;
Xqdot=0;
Xrdot=0;
Yudot=0;
Yvdot=-75.94;
Ywdot=0;
Ypdot=0;
Yqdot=0;
Yrdot=0;
Zudot=0;
Zvdot=0;
Zwdot=-111.47;
Zpdot=0;
Zqdot=0;
Zrdot=0;
Kudot=0;
Kvdot=0;
Kwdot=0;
Kpdot=-3.37;
Kqdot=0;
Krdot=0;
Mudot=0;
Mvdot=0;
Mwdot=0;
Mpdot=0;
Mqdot=-5.61;
Mrdot=0;
Nudot=0;
Nvdot=0;
Nwdot=0;
Npdot=0;
Nqdot=0;
Nrdot=-4;

Xu=0;
Yv=0;
Zw=0;
Kp=0;
Mq=0;
Nr=0;

Xuu=-34.55;
Yvv=-104.40;
Zww=-146.50;
Kpp=-0.68;
Mqq=-5.34;
Nrr=-3.07;


%govering equation is :M*Vdot+C*V+D*V+G=TAVc+TAVdisturbance
%TAVc is control force and moment in thruster and TAVdisturbance is disturbance force and moment
%M=Mrb+Madd is mass or inertia matrix

Mrb=[m 0 0 0 m*zg -m*yg;0 m 0 -m*zg 0 m*xg;0 0 m m*yg -m*xg 0;0 -m*zg m*yg Ixx -Ixy -Ixz ;m*zg 0 -m*xg -Ixy Iyy -Iyz;-m*yg m*xg 0 -Ixz -Iyz Izz];
Madd=-[Xudot Xvdot Xwdot Xpdot Xqdot Xrdot;Yudot Yvdot Ywdot Ypdot Yqdot Yrdot;Zudot Zvdot Zwdot Zpdot Zqdot Zrdot;Kudot Kvdot Kwdot Kpdot Kqdot Krdot;Mudot Mvdot Mwdot Mpdot Mqdot Mrdot;Nudot Nvdot Nwdot Npdot Nqdot Nrdot];
M=Mrb+Madd;
 

M;


%C=Crb+Cadd is coriolis matrix

a1=Xudot*u+ Xvdot*v +Xwdot*w+ Xpdot*p+ Xqdot*q+ Xrdot*r;
a2=Xudot*u+Yvdot*v+ Ywdot*w+ Ypdot*p+ Yqdot*q+ Yrdot*r;
a3=Xudot*u+Yvdot*v+ Zwdot*w+ Zpdot*p+ Zqdot*q+ Zrdot*r;
b1=Xudot*u+ Yvdot*v+ Zwdot*w+ Kpdot*p+ Kqdot*q+ Krdot*r;
b2=Xudot*u+ Yvdot*v+ Zwdot*w+ Kpdot*p+ Mqdot*q+ Mrdot*r;
b3=Xudot*u+ Yvdot*v+ Zwdot*w+ Kpdot*p+ Mqdot*q+ Nrdot*r;
Cadd=[0 0 0 0 -a3 a2;0 0 0 a3 0 -a1;0 0 0 -a2 a1 0;0 -a3 a2 0 -b3 b2;a3 0 -a1 b3 0 -b1;-a2 a1 0 -b2 b1 0];
C11=zeros(3,3);
C12=[m*(yg*q+zg*r) -m*(xg*q-w) -m*(xg*r+v);-m*(yg*q+w) m*(zg*r+xg*p) -m*(yg*r-u);-m*(zg*p-v) -m*(zg*q+u) m*(xg*p +yg*q)];
C22=[0 -Iyz*q-Ixz*p+Izz*r Iyz*r+Ixy*p-Iyy*q;Iyz*q+Ixz*p-Izz*r 0 -Ixz*r-Ixy*q+Ixx*p; -Iyz*r-Ixy*p+Iyy*q  Ixz*r+Ixy*q-Ixx*p 0];
Crb=[C11 C12;-C12' C22];
C=Crb+Cadd;





%D=D(linear drag coefficient)+D(quadratic drag coefficient) is hydrodynamic
%damping matrix 
%in this symolation  linear drag coefficientes are negligible

Dlinear=-[Xu 0 0 0 0 0;0 Yv 0 0 0 0;0 0 Zw 0 0 0 ;0 0 0 Kp 0 0 ;0 0 0 0 Mq 0;0 0 0 0 0 Nr];
Dquadratic=-[Xuu*abs(u) 0 0 0 0 0;0 Yvv*abs(v) 0 0 0 0;0 0 Zww*abs(w) 0 0 0 ;0 0 0 Kpp*abs(p) 0 0 ;0 0 0 0 Mqq*abs(q) 0;0 0 0 0 0 Nrr*abs(r)];
D=Dlinear+Dquadratic;


%G is gravity and bouancy vector that can be negligible
%because HM-AUV has natural bouancy and center of bouancy and  mass are coincide


%physical parameters of the vehicle
%w=yl=yr;
width=0.45;
%l=xt+xl=xt+xr;
l=0.885;
xl=0.135;
xr=0.135;
xt=0.75;

%defining the control force and moment of the thrusters as the inverse
%jacobian matrix of the error (e=current-desired position)
TAVcontrol=Jinv*e;

%according to the configuration of the vehicle, only the right and left
%thrusters contribute to x component of the force, all the three thrusters
%contribute to the z component of the force and only the tail thruster
%contributes to the y component of the force

fxthruster=TAVcontrol(1)/2;
fythruster=TAVcontrol(2);
fzthruster=TAVcontrol(3)/3;

%the added moment of thrusters in each direction should add up to zero and
%from this, the equations of added roll, pitch and yaw were drived as
%follows

fzaddroll2=TAVcontrol(4)/width;
fzaddroll1=TAVcontrol(4)/width;

fzaddpitch3=TAVcontrol(5)/l;
fzaddpitch2=TAVcontrol(5)/2*l;
fzaddpitch1=TAVcontrol(5)/2*l;

fxaddyaw2=TAVcontrol(6)/2*xl;
fxaddyaw1=TAVcontrol(6)/2*xr;


fxaddsway2=TAVcontrol(2)*xt/2*xl;
fxaddsway1=TAVcontrol(2)*xt/2*xr;

%due to the specific symmetry of the vehicle, the x, y, and z componets of
%the force generated by each thruster were drived
fzthruster1=fzthruster-fzaddroll1+fzaddpitch1;
fzthruster2=fzthruster+fzaddroll2+fzaddpitch2;
fzthruster3=fzthruster-fzaddpitch3;

fxthruster1=fxthruster+fxaddyaw1-fxaddsway1;
fxthruster2=fxthruster-fxaddyaw2+fxaddsway2;

fythruster3=fythruster;



%the magnitude of the total force of each thruster 
f1=sqrt(fxthruster1^2+fzthruster1^2);
f2=sqrt(fxthruster2^2+fzthruster2^2);
f3=sqrt(fythruster3^2+fzthruster3^2);

%govering equation is :M*Vdot+C*V+D*V+G=TAVcontrol and solving for Vdot aka
%acceleration which will be in need for the other blocks
Mfactor=inv(M)*TAVcontrol;
Cfactor=inv(M)*C*[u;v;w;p;q;r];
Dfactor=inv(M)*D*[u;v;w;p;q;r];

accelaration=Mfactor-Cfactor-Dfactor-inv(M)*tavdisturbance;

%calculating the theta angle for each thruster (rad)



thruster1teta=atan2(fzthruster1,fxthruster1);
thruster2teta=atan2(fzthruster2,fxthruster2);
thruster3teta=atan2(fzthruster3,fythruster3);





F=[f1;f2;f3];

