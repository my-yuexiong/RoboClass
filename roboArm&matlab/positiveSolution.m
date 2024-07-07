clc

L1 = 150;
L2 = 105;
L3 = 98;
L4 = 175;

angle0 = input('输入angle0：');
angle1 = input('输入angle1：');
angle2 = input('输入angle2：');
angle3 = input('输入angle3：');

seta0 = angle0 * pi / 180;
seta1 = angle1 * pi / 180;
seta2 = angle2 * pi / 180;
seta3 = angle3 * pi / 180;

d1 = L2 * cos(seta1);
d2 = L3 * cos(seta1 - seta2);
d3 = L4 * cos(seta2 + seta3 - seta1);
d = d1 + d2 + d3;

z1 = L3 * sin(seta1);
z2 = L3 * sin(seta1 - seta2);
z3 = L4 * sin(seta2 + seta3 - seta1);
z = L1 + z1 + z2 - z3;

x = d * sin(seta0);
y = d * cos(seta0);

O0 = [0,0];
O1 = [0,L1];
O2 = [d1,L1 + z1];
O3 = [d1 + d2,L1 + z1 + z2];
O4 = [d,z];

plot([O0(1), O1(1)], [O0(2), O1(2)], '-o');
hold on;
plot([O1(1), O2(1)], [O1(2), O2(2)], '-o');
hold on;
plot([O2(1), O3(1)], [O2(2), O3(2)], '-o');
hold on;
plot([O3(1), O4(1)], [O3(2), O4(2)], '-o');
hold on;

xlabel('d 轴');
ylabel('z 轴');
title('机械臂平面机构');
axis equal; 

fprintf('x = %f, y = %f, z = %f\n', x, y, z);


