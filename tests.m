up = fopen('left5.txt','r');
y = fscanf(up,'%f');
fclose(up);
for i=1:1:300
    x(i)=-5;
end
t=0:0.02:5.99;
hold on
p = plot(t,y,t,x);
title('Left')
xlabel('Time')
ylabel('Degrees')
grid on
grid minor

if x(1)>0
    yaxis1 = -5;
    yaxis2 = x(1)+5;
    ymotor = [-1 y(300)-1];
else
    yaxis2 = 5;
    yaxis1 = x(1)-5;
    ymotor = [1 y(300)+1];
end
axis([0,7,yaxis1,yaxis2]);

xmotor = [0.05 4];
str = {'Motor position = 750','Motor position = 595'};
text(xmotor,ymotor,str)

legend('Time Response','Reference Point')
text(6.05,y(300),'\leftarrow Steady State Error')


