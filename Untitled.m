M1o = 2800;
M2o = 800;
M3o = 3100;
M4o = 500;
PI = 3.14159265358979323846;
Theta0 = (63.43494882*PI)/180;

degreex = -30 : 1 : 30;
degreey = -30 : 1 : 30;

Thetax = (degreex*PI)/180;
Thetay = (degreey*PI)/180;

Lxx=sqrt((9-(5*cos(Theta0-abs(Thetax)))).^2+((5*sin(Theta0-abs(Thetax)))-2.5).^2)-0.045573;
Dxx=abs(Lxx-7);
Lyy=sqrt((9-(5*cos(Theta0-abs(Thetay)))).^2+((5*sin(Theta0-abs(Thetay)))-2.5).^2)-0.045573;
Dyy=abs(Lyy-7);
Lx1=Dxx;
Lx2=-Dxx;
Ly1=Dyy;
Ly2=-Dyy;

rmotor = 4;
umotor1 = (Lx1*180)/(PI*rmotor);
umotor3 = (Lx2*180)/(PI*rmotor);
umotor2 = (Ly1*180)/(PI*rmotor);
umotor4 = (Ly2*180)/(PI*rmotor);

for i = 1 : 30
    M1(i) = M1o - fix(abs((4096*umotor1(i))/360));
    M3(i) = M3o + fix(abs((4096*umotor3(i))/360));
end
    
for i = 31 : 61
    M1(i) = M1o + fix(abs((4096*umotor1(i))/360));
    M3(i) = M3o - fix(abs((4096*umotor3(i))/360));
end
    
for j = 1 : 30
    M2(j) = M2o - fix(abs((4096*umotor2(j))/360));
    M4(j) = M4o + fix(abs((4096*umotor4(j))/360));
end
    
for j = 31 : 61
    M2(j) = M2o + fix(abs((4096*umotor2(j))/360));
    M4(j) = M4o - fix(abs((4096*umotor4(j))/360));
end

hold on
subplot(2,1,1);
plot(degreex,M1,degreex,M3);
title('Degrees - Motor Position (Up and Down Rotation)')
xlabel('Degrees')
ylabel('Motor Position')
grid on
grid minor

axis([-40,40,2000,4000]);
legend('Up Puller Servo','Down Puller Servo')

subplot(2,1,2);
plot(degreey,M4,degreey,M2);
title('Degrees - Motor Position (Right and Left Rotation)')
xlabel('Degrees')
ylabel('Motor Position')
grid on
grid minor

axis([-40,40,-300,1700]);
legend('Right Puller Servo','Left Puller Servo')