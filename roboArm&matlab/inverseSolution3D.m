clc;
clear;

L1 = 150;
L2 = 105;
L3 = 98;
L4 = 175;

% 输入已知的末端坐标和pitch
x = input('输入末端x坐标：');
y = input('输入末端y坐标：');
z = input('输入末端z坐标：');
pitch = input('输入pitch角度(度)：');

% 将pitch转换为弧度
pitch = pitch * pi / 180;

% 计算seta0
seta0 = atan2(x, y);

% 计算d
d = sqrt(x^2 + y^2);

% 根据几何关系和已知的z坐标，求解seta1, seta2
% 从z轴分量出发，计算总高度差
delta_z = z - L1;

% 使用数值解法来求解seta1和seta2
syms seta1 seta2;
eq1 = L2*sin(seta1) + L3*sin(seta1 - seta2) - L4*sin(pitch) == delta_z;
eq2 = L2*cos(seta1) + L3*cos(seta1 - seta2) + L4*cos(pitch) == d;

% 使用求解器求解方程组
sol = vpasolve([eq1, eq2], [seta1, seta2]);
seta1 = double(sol.seta1);
seta2 = double(sol.seta2);

% 计算seta3
seta3 = pitch + seta2 - seta1;

% 将角度转换为度数输出
angle0 = seta0 * 180 / pi;
angle1 = seta1 * 180 / pi;
angle2 = seta2 * 180 / pi;
angle3 = seta3 * 180 / pi;

fprintf('angle0 = %f 度\n', angle0);
fprintf('angle1 = %f 度\n', angle1);
fprintf('angle2 = %f 度\n', angle2);
fprintf('angle3 = %f 度\n', angle3);
