%clear screen, working memory, and close all old figures
clc
clear
close all

x1=linspace(0, 65);
x2=(0:3.61:4.17*15.7)

y1=2.085*sin(0.435*x1);
y2=2.085*sin(0.435*x2);
plot(x2,y2, '.r', 'MarkerSize', 20);sssssss
hold on;
plot(x1, y1, '--k');

title('Desired Position when More Pressure on Odd Segments');
xlabel('Position (inch)');
ylabel('Segment Height(inch)');

legend('Segment Joints', 'Sinusoidal Model', 'Location', 'best');

figure(2);

y3=-2.085*sin(0.435*x1);
y4=-2.085*sin(0.435*x2);
plot(x2,y4, '.r', 'MarkerSize', 20);
hold on;
plot(x1, y3, '--k');

title('Desired Position when More Pressure on Even Segments');
xlabel('Position (inch)');
ylabel('Segment Height(inch)');

legend('Segment Joints', 'Sinusoidal Model', 'Location', 'best');


