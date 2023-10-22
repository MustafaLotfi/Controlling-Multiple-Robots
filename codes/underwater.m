function [xe,ye,tawu,tawv,udot,vdot,rdot,xdot,ydot,psidot,ev,eu,gama1,gama2]= fcn(t,u,r,v,x,y,psi,inteu,intev,gama1_d,gama2_d)
%#codegen
 
m=30.48;
lx=2;
ly=2;
Iz=3.45;
xu=-8.8065;
yv=-65.5457;
nr=-6.7352;
xudot=-0.93;
yvdot=-35.5;
nrdot=-35.5;
m1=1/(m-xudot);m2=1/(m-yvdot);m3=1/(Iz-nrdot);
a12=yvdot-xudot;a13=xudot-m;a23=m-yvdot;
 
w1=0.5;
w2=3;
landa1=2;
landa2=7;
landa3=8;
 
kx=0.5;
ky=0.5;
k1=0;
k2=0;
 
% x_d=10*cos(t);y_d=10*sin(t);
% xdot_d=-10*sin(t);ydot_d=10*cos(t);
% xddot_d=-10*cos(t);yddot_d=-10*sin(t);
% xdddot_d=10*sin(t);ydddot_d=-10*cos(t);


xdot=u*cos(psi)-v*sin(psi);
ydot=u*sin(psi)+v*cos(psi);
psidot=r;
xe=x-x_d;
ye=y-y_d;
% xye_dot=[cos(psi) -sin(psi);sin(psi) cos(psi)]*[u;v]-[xdot_d;ydot_d];
% xe_dot=xye_dot(1);
% ye_dot=xye_dot(2);
uvd=[cos(psi) sin(psi);-sin(psi) cos(psi)]*[xdot_d+lx*tanh(-kx*xe/lx);ydot_d+ly*tanh(-ky*ye/ly)];
ud=uvd(1);
vd=uvd(2);
xe_dot=lx*tanh(-kx*xe/lx);
ye_dot=ly*tanh(-ky*ye/ly);
eu=u-ud;
ev=v-vd;
gama1=kx*xe_dot*(sech(-kx*xe/lx))^2;
gama2=ky*ye_dot*(sech(-ky*ye/ly))^2;
uvd_dot=r*[-sin(psi) cos(psi);-cos(psi) -sin(psi)]*[xdot_d+xe_dot;ydot_d+ye_dot]+...
    [cos(psi) sin(psi);-sin(psi) cos(psi)]*[xddot_d-gama1;yddot_d-gama2];
ud_dot=uvd_dot(1);
vd_dot=uvd_dot(2);
Gama=-xdddot_d*sin(psi)+ydddot_d*cos(psi)-xddot_d*r*cos(psi)-yddot_d*r*sin(psi)-...
    ud_dot*r+gama1*r*cos(psi)+gama2*r*sin(psi)+gama1_d*sin(psi)-gama2_d*cos(psi);
ev_dot=m2*(yv*v+a13*u*r)-vd_dot;
s1=eu+landa1*inteu;
s2=ev+landa2*intev;
rdot=m3*(nr*r+a12*u*v);

tu1=-xu*u-a23*v*r+(1/m1)*ud_dot-(1/m1)*landa1*eu;
tu2=(1/m1)*(-k1*s1-w1*tanh(s1));



tawu=tu1+tu2;

udot=m1*(xu*u+a23*v*r+tawu);
tv1=-yv*v-a13*u*r+(1/m2)*vd_dot-(1/m2)*landa2*ev;
tv2=(1/m2)*(-k2*s2-w2*tanh(s2));
   
tawv=tv1+tv2;

vdot=m2*(yv*v+a13*u*r+tawv);