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

m = d - L4 * cos(pitch);
n = z - L1 + L4 * sin(pitch);

seta2 = acos((m^2 + n^2 - L2^2 - L3^2) / (2 * L2 * L3));

seta2 = seta2 / pi * 180;