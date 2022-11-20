%This function solves the direct kinematics of hexapod mechanism. The function takes the initial guess (P0) 
%(position and orientation of the upper platform) together with a vector containing leg 
%lengths (lg)as the input and returns the 
%approximation of the upper platform's position and orientation with the 
%specified precision/accuracy. The precision is considered to be 0.000001 mm.
function p=ForwardKinematics(P0,lg)
t=1;
Rm=250.9339/2;
Rf=634.2376/2;
alpha=40*pi/180;
beta=85*pi/180;
P(:,1)=P0;
i=2;
while t>0.000001
    X=P(1:3,i-1);
    teta=P(4:6,i-1);
    J=jacobianV(P(:,i-1));
    [L,l,n]=pod(P(:,i-1));
    Rp=[1,0,0,0,0,0;
        0,1,0,0,0,0;
        0,0,1,0,0,0;
        0,0,0,1,0,sin(teta(2)*pi/180);
        0,0,0,0,cos(teta(1)*pi/180),-sin(teta(1)*pi/180)*cos(teta(2)*pi/180);
        0,0,0,0,sin(teta(1)*pi/180),cos(teta(1)*pi/180)*cos(teta(2)*pi/180)];
% Rp is used to represent the Euler angles' error in the base frame. 
    JRp=J*Rp;
    P(:,i)=P(:,i-1)-inv(JRp)*(l'-lg);
    t=norm(P(:,i)-P(:,i-1),2);
    i=i+1;
end
p=P(:,end);