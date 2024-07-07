%赋予输入值
l1=150;
l2=105;
l3=98;
l4=175;
% l1=153;
% l2=106;
% l3=100;
% l4=166;
IN_theta = [53/180*pi,72/180*pi,31/180*pi,43/180*pi,0/180*pi];
%建立D-H参数表
C_a = [0,0,l2,l3,0,0];
C_d = [l1,0,0,0,0,l4];
C_alpha = [0,-pi/2,0,0,-pi/2,0];
C_theta = [IN_theta(1),IN_theta(2),IN_theta(3),IN_theta(4)-pi/2,IN_theta(5),0];
% 参数矩阵取名为MDH
MDH = [C_theta(1) C_d(1) C_a(1) C_alpha(1);
C_theta(2) C_d(2) C_a(2) C_alpha(2); 
C_theta(3) C_d(3) C_a(3) C_alpha(3);
C_theta(4) C_d(4) C_a(4) C_alpha(4);
C_theta(5) C_d(5) C_a(5) C_alpha(5);
C_theta(6) C_d(6) C_a(6) C_alpha(6)];
T01=[cos(MDH(1,1)) -sin(MDH(1,1)) 0 MDH(1,3);
sin(MDH(1,1))*cos(MDH(1,4)) cos(MDH(1,1))*cos(MDH(1,4)) -sin(MDH(1,4)) -sin(MDH(1,4))*MDH(1,2);
sin(MDH(1,1))*sin(MDH(1,4)) cos(MDH(1,1))*sin(MDH(1,4)) cos(MDH(1,4)) cos(MDH(1,4))*MDH(1,2);
0 0 0 1];
T12=[cos(MDH(2,1)) -sin(MDH(2,1)) 0 MDH(2,3);
sin(MDH(2,1))*cos(MDH(2,4)) cos(MDH(2,1))*cos(MDH(2,4)) -sin(MDH(2,4)) -sin(MDH(2,4))*MDH(2,2);
sin(MDH(2,1))*sin(MDH(2,4)) cos(MDH(2,1))*sin(MDH(2,4)) cos(MDH(2,4)) cos(MDH(2,4))*MDH(2,2);
0 0 0 1];
T23=[cos(MDH(3,1)) -sin(MDH(3,1)) 0 MDH(3,3);
sin(MDH(3,1))*cos(MDH(3,4)) cos(MDH(3,1))*cos(MDH(3,4)) -sin(MDH(3,4)) -sin(MDH(3,4))*MDH(3,2);
sin(MDH(3,1))*sin(MDH(3,4)) cos(MDH(3,1))*sin(MDH(3,4)) cos(MDH(3,4)) cos(MDH(3,4))*MDH(3,2);
0 0 0 1];
T34=[cos(MDH(4,1)) -sin(MDH(4,1)) 0 MDH(4,3);
sin(MDH(4,1))*cos(MDH(4,4)) cos(MDH(4,1))*cos(MDH(4,4)) -sin(MDH(4,4)) -sin(MDH(4,4))*MDH(4,2);
sin(MDH(4,1))*sin(MDH(4,4)) cos(MDH(4,1))*sin(MDH(4,4)) cos(MDH(4,4)) cos(MDH(4,4))*MDH(4,2);
0 0 0 1];
T45=[cos(MDH(5,1)) -sin(MDH(5,1)) 0 MDH(5,3);
sin(MDH(5,1))*cos(MDH(5,4)) cos(MDH(5,1))*cos(MDH(5,4)) -sin(MDH(5,4)) -sin(MDH(5,4))*MDH(5,2);
sin(MDH(5,1))*sin(MDH(5,4)) cos(MDH(5,1))*sin(MDH(5,4)) cos(MDH(5,4)) cos(MDH(5,4))*MDH(5,2);
0 0 0 1];
T56=[cos(MDH(6,1)) -sin(MDH(6,1)) 0 MDH(6,3);
sin(MDH(6,1))*cos(MDH(6,4)) cos(MDH(6,1))*cos(MDH(6,4)) -sin(MDH(6,4)) -sin(MDH(6,4))*MDH(6,2);
sin(MDH(6,1))*sin(MDH(6,4)) cos(MDH(6,1))*sin(MDH(6,4)) cos(MDH(6,4)) cos(MDH(6,4))*MDH(6,2);
0 0 0 1];

T06 = round(T01*T12*T23*T34*T45*T56);