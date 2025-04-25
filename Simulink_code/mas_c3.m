function [sys,x0,str,ts]=s_function(t,x,u,flag)
switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1,
    sys=mdlDerivatives(t,x,u);
case 3,
    sys=mdlOutputs(t,x,u);
case {2, 4, 9 }
    sys = [];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end


function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 8;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys=simsizes(sizes);
x0=[];
str=[];
ts=[0 0];

function sys=mdlOutputs(t,x,u)
sys = zeros(1,2);
e = [u(1) u(2)]';e_dot = [u(3) u(4)]';
dq1 = u(5); dq2 = u(6);
dq = [u(5) u(6)]'; q1 = u(7);
q2 = u(8);

m1=0.23;m2=0.46;I1 = 0.03; I2 = 0.06;  l1=0.3;   l2=0.3; a1 = 0.1; a2 =0.1;
g=9.8; kg=1; 

H=[I1+I2+l1*l1*m1+2*a1*l2*m2*cos(q2)+m2*(a1^2+l2^2),I2+l2*l2*m2+a1*l2*m2*cos(q2);
    I2+l2*l2*m2+a1*l2*m2*cos(q2),I2+l2*l2*m2];
C=[-2*l1*l2*m2*sin(q2)*dq2,-l1*l2*m2*sin(q2)*dq2;
    l1*l2*m2*sin(q2)*dq2,0];
G=[m2*l2*g*cos(q1+q2)+(m1*l1+m2*a1)*g*cos(q1);
    m2*l2*g*cos(q1+q2)];    
kps = [1,0;0,1]*100; kvs = [1,0;0,1]*10;


% KK2 = [k2,zerohold;zerohold,k2];
% result = kds*r2+kbs*r1+G-kb*dq;
result =  kps*e+kvs*e_dot+G+C*dq;
sys(1)=result(1);
sys(2)=result(2);