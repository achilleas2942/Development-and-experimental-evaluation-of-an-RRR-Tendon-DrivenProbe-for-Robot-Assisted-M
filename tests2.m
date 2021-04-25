up = fopen('down20-0-10.txt','r');
y = fscanf(up,'%f');
y1 = y(1:300);
y2 = y(301:600)+y(300);
y3 = y(601:900)+y(600)+y(300);
fclose(up);
for i=1:1:300
    x(i)=20;
end
for i=301:1:600
    x(i)=0;
end
for i=601:1:900
    x(i)=10;
end
y(1:300)=y1;
y(301:600)=y2;
y(601:900)=y3;
t=0:0.01:8.99;
hold on
p = plot(t,y,t,x);
title('Down')
xlabel('Time')
ylabel('Degrees')
grid on
grid minor

if x(1)>0
    yaxis1 = -5;
    yaxis2 = x(1)+5;
    ymotor = [-1 y(300)-1 -1 y(800)-1];
else
    yaxis2 = 5;
    yaxis1 = x(1)-5;
    ymotor = [1 y(300)+1 1 y(800)+1];
end
axis([0,10,yaxis1,yaxis2]);

xmotor = [0.05 1 4 7];
str = {'Motor position = 3200','Motor position = 3797','Motor position = 3200','Motor position = 3513'};
text(xmotor,ymotor,str)

legend('Time Response','Reference Point')
text(3.8,y(300),'\leftarrow Steady State Error')
text(7,y(600),'\leftarrow Steady State Error')
text(9.1,y(900),'\leftarrow Steady State Error')
