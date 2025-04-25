function [sys,x0,str,ts]=s_function(t,x,u,flag)  %主手1的动力学解算
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
sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys=simsizes(sizes);

x0=[0.42 0 2.85 0];
str=[];
ts=[0 0];

function sys=mdlDerivatives(t,x,u)
tol=[u(1);u(2)];
q1=x(1);dq1=x(2);
q2=x(3);dq2=x(4);

m1=0.12;m2=0.14;I1 = 0.01; I2 = 0.02;  l1=0.3;   l2=0.3; a1 = 0.1; a2 =0.1;
g=9.8; 


H=[I1+I2+l1*l1*m1+2*a1*l2*m2*cos(q2)+m2*(a1^2+l2^2),I2+l2*l2*m2+a1*l2*m2*cos(q2);
    I2+l2*l2*m2+a1*l2*m2*cos(q2),I2+l2*l2*m2];
C=[-2*l1*l2*m2*sin(q2)*dq2,-l1*l2*m2*sin(q2)*dq2;
    l1*l2*m2*sin(q2)*dq2,0];
G=[m2*l2*g*cos(q1+q2)+(m1*l1+m2*a1)*g*cos(q1);
    m2*l2*g*cos(q1+q2)];

S=H\(tol-C*[dq1;dq2]-G);
sys(1)=x(2);
sys(2)=S(1);
sys(3)=x(4);
sys(4)=S(2);

function sys=mdlOutputs(t,x,u)
sys = zeros(1,6);
tol=[u(1);u(2)];


q1=x(1);dq1=x(2);
q2=x(3);dq2=x(4);

m1=0.12;m2=0.14;I1 = 0.01; I2 = 0.02;  l1=0.3;   l2=0.3; a1 = 0.1; a2 =0.1;
g=9.8;  

H=[I1+I2+l1*l1*m1+2*a1*l2*m2*cos(q2)+m2*(a1^2+l2^2),I2+l2*l2*m2+a1*l2*m2*cos(q2);
    I2+l2*l2*m2+a1*l2*m2*cos(q2),I2+l2*l2*m2];
C=[-2*l1*l2*m2*sin(q2)*dq2,-l1*l2*m2*sin(q2)*dq2;
    l1*l2*m2*sin(q2)*dq2,0];
G=[m2*l2*g*cos(q1+q2)+(m1*l1+m2*a1)*g*cos(q1);
    m2*l2*g*cos(q1+q2)];

S=H\(tol-C*[dq1;dq2]-G);
sys(1)=x(1);
sys(2)=x(2);
sys(3)=S(1);
sys(4)=x(3);
sys(5)=x(4);
sys(6)=S(2);