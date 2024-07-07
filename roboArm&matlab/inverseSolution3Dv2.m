clc;
clear;

L1 = 150;
L2 = 105;
L3 = 98;
L4 = 175;

x = input('输入末端x坐标：');
y = input('输入末端y坐标：');
z = input('输入末端z坐标：');
pitch = input('输入pitch角度(度)：');

pitch = pitch * pi / 180;

seta0 = atan2(x, y);

d = sqrt(x^2 + y^2);

delta_z = z - L1;

Lx = L4 * cos(pitch);
Ly = L4 * sin(pitch);
O3 = [d - Lx, z + Ly];
O1 = [0, L1];
d13 = sqrt((O3(1) - O1(1))^2 + (O3(2) - O1(1))^2);

fun = @(x) [L2*sin(x(1)) + L3*sin(x(1) - x(2)) - L4*sin(pitch) - delta_z;
            L2*cos(x(1)) + L3*cos(x(1) - x(2)) + L4*cos(pitch) - d];

x0 = [0; 0]; 

options = optimoptions('fsolve', 'Display', 'iter');
[x, fval, exitflag] = fsolve(fun, x0, options);

seta1 = x(1);
seta2 = x(2);

x0 = [pi/2; 0]; 
[x, fval, exitflag] = fsolve(fun, x0, options);
seta1 = x(1);
seta2 = x(2);

seta3 = pitch - seta2 + seta1;

angle0 = seta0 * 180 / pi;
angle1 = seta1 * 180 / pi;
angle2 = seta2 * 180 / pi;
angle3 = seta3 * 180 / pi;

fprintf('angle0 = %f 度\n', angle0);
fprintf('angle1 = %f 度\n', angle1);
fprintf('angle2 = %f 度\n', angle2);
fprintf('angle3 = %f 度\n', angle3);
