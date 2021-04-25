x1=2900:(3730-2900)/700:3729;
up = fopen('up.txt','r');
text1 = fscanf(up,'%f');
fclose(up);
x2=3200:(4045-3200)/700:4044;
down = fopen('down.txt','r');
text2 = fscanf(up,'%f');
fclose(down);
x3=750:(-47-750)/700:-46;
left = fopen('left.txt','r');
text3 = fscanf(left,'%f');
fclose(left);
x4=470:(-369-470)/700:-368;
right = fopen('right.txt','r');
text4 = fscanf(right,'%f');
fclose(right);
subplot(2,2,1);
plot(x1,text1);
title('Subplot 1: Up')
xlabel('Motor position')
ylabel('Degrees')
grid on
grid minor
subplot(2,2,2);
plot(x2,text2);
title('Subplot 2: Down')
xlabel('Motor position')
ylabel('Degrees')
grid on
grid minor
subplot(2,2,3);
plot(x3,text3);
title('Subplot 3: Left')
xlabel('Motor position')
ylabel('Degrees')
grid on
grid minor
subplot(2,2,4);
plot(x4,text4);
title('Subplot 4: Right')
xlabel('Motor position')
ylabel('Degrees')
grid on
grid minor