up = fopen('up15right10.txt','r');
y = fscanf(up,'%f');
fclose(up);
for i=1:1:300
    x1(i) = 10;
    y1(i) = y(2*(i-1)+1);
end
for j=1:1:300
    x2(j) = -15;
    y2(j) = y(2*j);
end
t=0:0.02:5.99;
hold on
subplot(2,1,1);
plot(t,y1,t,x1);
title('Right')
xlabel('Time')
ylabel('Degrees')
grid on
grid minor

axis([0,7,-5,15]);

xmotor1 = [0.05 4];
ymotor1 = [1 y1(300)+1];
str1 = {'Motor position = 470','Motor position = 157'};
text(xmotor1,ymotor1,str1)

legend('Time Response','Reference Point')
text(6.1,y1(300),'\leftarrow Steady State Error')

subplot(2,1,2);
plot(t,y2,t,x2);
title('Up')
xlabel('Time')
ylabel('Degrees')
grid on
grid minor

axis([0,7,-20,5]);

xmotor2 = [0.05 4];
ymotor2 = [1 x2(300)+1];
str2 = {'Motor position = 2900','Motor position = 3296'};
text(xmotor2,ymotor2,str2)

legend('Time Response','Reference Point')
text(6.1,y2(300),'\leftarrow Steady State Error')
