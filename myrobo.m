%% Project 1
syms u1;
syms u2;
syms u3;
L(1) = Link([u1,  0, 0, -pi/2, 0], 'standard');
L(2) = Link([u2,   0, 0,  pi/2, 0], 'standard');
L(3) = Link([u3, 26, 0,  0, 0], 'standard');
puma560=SerialLink(L,'name','myrobo');
A1=L(1).A(u1);
A2=L(2).A(u2);
A3=L(3).A(u3);
A=A1*A2*A3;

%%
C=A();
k=30*pi/180;
j=30*pi/180;
i=60*pi/180;
D=subs(C,{u1,u2,u3},{k,j,i});
disp(D)

%%
u1=30;
u2=45;
u3=0;
Li=(53)^(1/2);
Lo=10;
Lxx=((8.5-((11.25)^(1/2)*cosd(63.43494882-u2)))^2+(((11.25)^(1/2)*sind(63.43494882-u2))-1)^2)^(1/2);
Lyx=(((53)^(1/2)+1.5-1.5*cosd(u2))^2+(1.5*sind(u2))^2)^(1/2);
Dxx=abs(Lxx-Li);
if abs(Dxx)<1e-6
   Dxx=0;
end
Dyx=abs(Lyx-Li);
if abs(Dyx)<1e-6
   Dyx=0;
end
Lyy=((8.5-((11.25)^(1/2)*cosd(63.43494882-u3)))^2+(((11.25)^(1/2)*sind(63.43494882-u3))-1)^2)^(1/2);
Lxy=(((53)^(1/2)+1.5-1.5*cosd(u3))^2+(1.5*sind(u3))^2)^(1/2);
Dyy=abs(Lyy-Li);
if abs(Dyy)<1e-6
   Dyy=0;
end
Dxy=abs(Lxy-Li);
if abs(Dxy)<1e-6
   Dxy=0;
end
Lxz=(10^2+(4.5*(1-cosd(u1)))^2)^(1/2);
Dxz=abs(Lxz-Lo);
Lyz=Lxz;
Dyz=abs(Lyz-Lo);
Lzz=3.5*((u1*pi)/180);
Lx=Dxx+Dxy+Dxz;
Ly=Dyx+Dyy+Dyz;
Lz=Lzz;
%%
C=A(1:3,4);
n=1;
i=0;
for j=-30*pi/180:1*pi/180:30*pi/180;
    for k=0*pi/180:10*pi/180:180*pi/180;
            D=subs(C,{u1,u2,u3},{k,j,i});
            x(n)=D(1);
            y(n)=D(2);
            z(n)=D(3);
            n=n+1;
    end
end
plot3(x,y,z)
%%
S=[];
for m=1:length(x)
    S(m,1)=x(m);
    S(m,2)=y(m);
    S(m,3)=z(m);
end
l=convhulln(S); 
plot3(x(l),y(l),z(l))
%%
B1=8;
B2=12;
t1=6;
t2=3;
t=0:0.1:600;
u1d=(pi/180)*(B1*sin(2*pi*t/t1));
u2d=(pi/180)*(B2*sin(2*pi*t/t2));
figure
subplot(2,1,1)
plot(t,u1d)
xlabel('time')
title('u1d')
subplot(2,1,2)
plot(t,u2d)
xlabel('time')
title('u2d')
%%
u3d=0;
du1d=[diff(u1d) 0];
du2d=[diff(u2d) 0];
du3d=[diff(u3d) 0];
ddu1d=[diff(u1d,2) 0 0];
ddu2d=[diff(u2d,2) 0 0];
ddu3d=[diff(u3d,2) 0 0];
N=120;
ui=[(pi/180)*(-160) (pi/180)*(-225) (pi/180)*(-45)]';
dui=[0;0;0];
udi=[u1d(1); u2d(1); u3d(1)];
dudi=[du1d(1); du2d(1); du3d(1)];
ei=udi-ui;
dei=dudi-dui;
x_initial=[ei;dei];
Z=zeros(3,3);
I=eye(3,3);
KP=diag([0.01,0.01,0.01]);
KD=diag([0.1,0.01,0.1]);
A_CL=[Z, I; -KP -KD];
G=ss(A_CL,[0; 0; 0; 0; 0; 0], eye(6), [0; 0; 0; 0; 0; 0]);
ERROR=lsim(G,zeros(1,6001),t,x_initial);
theta1=u1d-ERROR(:,1)';
theta2=u2d-ERROR(:,2)';
theta3=u3d-ERROR(:,3)';
th_dot1= du1d-ERROR(:,4)';
th_dot2= du2d-ERROR(:,5)';
figure
subplot(2,1,1)
plot(t,u1d,t,theta1)
xlabel('time')
subplot(2,1,2)
plot(t,u2d,t,theta2)
xlabel('time')

%%
    Thetax = 30;
    Thetay = 45;
    Thetaz = 20;
    Lxi = (53)^(1/2);
    Lyi = (53)^(1/2);
    Lzi = 10;
    L0 = 10;
    Theta0 = 63.43494882;
    Lxx=((8.5-((11.25)^(1/2)*cosd(Theta0-Thetax)))^2+(((11.25)^(1/2)*sind(Theta0-Thetax))-1)^2)^(1/2);
    Dxx=abs(Lxx-Lxi);
    if abs(Dxx)<1e-6
        Dxx=0;
    end
    Lxy=((Lxi+1.5-1.5*cosd(Thetay))^2+(1.5*sind(Thetay))^2)^(1/2);
    Dxy=abs(Lxy-Lxi);
    if abs(Dxy)<1e-6
        Dxy=0;
    end
    Lxz=(L0^2+(4.5*(1-cosd(Thetaz)))^2)^(1/2);
    Dxz=abs(Lxz-L0);
    Lyy=((8.5-((11.25)^(1/2)*cosd(Theta0-Thetay)))^2+(((11.25)^(1/2)*sind(Theta0-Thetay))-1)^2)^(1/2);
    Dyy=abs(Lyy-Lyi);
    if abs(Dyy)<1e-6
        Dyy=0;
    end
    Lyx=((Lyi+1.5-1.5*cosd(Thetax))^2+(1.5*sind(Thetax))^2)^(1/2);
    Dyx=abs(Lyx-Lyi);
    if abs(Dyx)<1e-6
        Dyx=0;
    end
    Lyz=Lxz;
    Dyz=abs(Lyz-L0);
    Lzz=3.5*((Thetaz*pi)/180);
    Lx=Dxx+Dxy+Dxz;
    Ly=Dyx+Dyy+Dyz;
    Lz=Lzz;