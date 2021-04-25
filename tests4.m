up = fopen('z.txt','r');
y = fscanf(up,'%f');
fclose(up);
for i=1:1:30
    x(i)=0;
end
for i=30:1:83
    x(i)=57;
end
for i=83:1:120
    x(i)=116;
end
for i=120:1:219
    x(i)=186;
end
for i=219:1:320
    x(i)=257;
end
for i=320:1:432
    x(i)=27;
end
t1=0:0.01:6.99;
t2=0:0.01:4.31;
hold on
p = plot(t1,y,t2,x);
title('Z')
xlabel('Time')
ylabel('Degrees')
grid on
grid minor

yaxis1 = -20;
yaxis2 = 280;
ymotor = [-1 y(300)-1];
axis([0,8,yaxis1,yaxis2]);

legend('Time Response','Reference Point')