%Inverse Kinematics

function [L,l,n,s,u,R]=pod(P)
Rm=250.9339/2;
Rf=634.2376/2;
alpha=40*pi/180;
beta=85*pi/180;
Xp=P(1:3,1);
a=P(4)*pi/180;
b=P(5)*pi/180;
c=P(6)*pi/180;
%XYZ Rotation Matrix (R)
R1=[1,0,0;0,cos(a),-sin(a);0,sin(a),cos(a)];
R2=[cos(b),0,sin(b);0,1,0;-sin(b),0,cos(b)];
R3=[cos(c),-sin(c),0;sin(c),cos(c),0;0,0,1];
R=R1*R2*R3;


s=[Rm*cos(beta/2),-Rm*sin(pi/6-beta/2),-Rm*sin(pi/6+beta/2),-Rm*cos(pi/3-beta/2),-Rm*cos(pi/3+beta/2),Rm*cos(beta/2);
    Rm*sin(beta/2),Rm*cos(pi/6-beta/2),Rm*cos(pi/6+beta/2),-Rm*sin(pi/3-beta/2),-Rm*sin(pi/3+beta/2),-Rm*sin(beta/2);
    0,0,0,0,0,0];


u=[Rf*cos(alpha/2),-Rf*sin(pi/6-alpha/2),-Rf*sin(pi/6+alpha/2),-Rf*cos(pi/3-alpha/2),-Rf*cos(pi/3+alpha/2),Rf*cos(alpha/2);
    Rf*sin(alpha/2),Rf*cos(pi/6-alpha/2),Rf*cos(pi/6+alpha/2),-Rf*sin(pi/3-alpha/2),-Rf*sin(pi/3+alpha/2),-Rf*sin(alpha/2);
    0,0,0,0,0,0];


for i=1:6
    L(:,i)=R*s(:,i)+Xp-u(:,i);
    l(i)=norm(L(:,i),2);
    n(:,i)=L(:,i)/l(i);
end
%L is the vector of each pod/leg and l is a vector containing the lengths of
%pods