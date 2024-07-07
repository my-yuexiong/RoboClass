clc;
L1 = 150;
L2 = 105;
L3 = 98;
L4 = 175;

seta0 = input("请输入seta0: ");
seta1 = input("请输入seta1: ");
seta2 = input("请输入seta2: ");
seta3 = input("请输入seta3: ");

d1 = L2 * cos(seta1);
d2 = L3 * cos(seta1 - seta2);
d3 = L4 * sin(pi / 2 - seta3 + seta1 -seta2);

d = d1 + d2 + d3;

z1 = L2 * sin(seta2);
z2 = L3 * sin(seta1 - seta2);
z3 = L4 * cos(pi / 2 - seta3 + seta1 - seta2);
z = L1 + z1 + z2 + z3;

x = d * sin(seta0);
y = d * cos(seta0);

disp(x);
disp(y);
disp(z);