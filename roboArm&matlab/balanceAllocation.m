x1 = input('请输入第一个点的x坐标: ');
y1 = input('请输入第一个点的y坐标: ');
z1 = input('请输入第一个点的z坐标: ');

x2 = input('请输入第二个点的x坐标: ');
y2 = input('请输入第二个点的y坐标: ');
z2 = input('请输入第二个点的z坐标: ');

point1 = [x1, y1, z1];
point2 = [x2, y2, z2];

num_points = 11; 
x = linspace(x1, x2, num_points);
y = linspace(y1, y2, num_points);
z = linspace(z1, z2, num_points);

interp_points = [x', y', z'];

figure;
plot3(x, y, z, 'o-');
hold on;
plot3(x1, y1, z1, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % 标记第一个点
plot3(x2, y2, z2, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); % 标记第二个点
hold off;

grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('三维空间中的两点及其插补的线段');
legend('插补线段', '起点', '终点');

disp('插补点的坐标为:');
disp(interp_points);
