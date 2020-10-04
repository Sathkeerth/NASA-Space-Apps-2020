% Robot values
Sb = 0.640; % base equilateral triangle side
Sp = 0.100; % platform equilateral triangle side
L = 0.300; % upper legs length
Xb = 0.180; % planar distance from {0} to near base side
Yb = 0.360; % planar distance from {0 } to a base vertex
Xp = 0.030; % planar distance from {P} to near platform side
Yp = 0.035; % planar distance from {P} to a platform vertex
P = [0.0; 0.013362; -0.758460]; %Coordinates
T1 = 0.264188; %Angle upper arm 1
T2 = 0.220808; %Angle upper arm 2
T3 = 0.220808; %Angle upper arm 3
% fixed-base revolute joint points B (constant in the base frame B)
B_1 = [0,-Xb,0];
B_2 = [(sqrt(3)/2)*Xb, 0.5*Xb,0];
B_3 = [-(sqrt(3)/2)*Xb, 0.5*Xb,0];
%edges of the base triangle
b_1 = [Sb/2, -Xb, 0];
b_2 = [0, Yb, 0];
b_3 = [-Sb/2, -Xb, 0];
%knee points of the delta robot
A_1 = [0;-Xb-L*cos(T1); -L*sin(T1)];
A_2 = [sqrt(3)/2 * (Xb+L*cos(T2)); 1/2 * (Xb+L*cos(T2)); -L*sin(T2)];
A_3 = [-sqrt(3)/2 * (Xb+L*cos(T3)); 1/2 * (Xb+L*cos(T3)); -L*sin(T3)];
% platform-fixed U-joint connection (constant in the base frame P)
P_1 = [0; -Yp; 0];
P_2 = [Sp/2; Xp; 0];
P_3 = [-Sp/2; Xp; 0];
% platform-fixed U-joint connection (when P is not located in {0 0 0})
Pf_1 = P+P_1;
Pf_2 = P+P_2;
Pf_3 = P+P_3;
% linspace--> to create the points to plot (lines)
%base triangle
BT_1(:,1) = linspace(b_1(1),b_2(1));
BT_1(:,2) = linspace(b_1(2),b_2(2)); % triangle side 1 components
BT_1(:,3) = linspace(b_1(3),b_2(3));
BT_2(:,1) = linspace(b_2(1),b_3(1));
BT_2(:,2) = linspace(b_2(2),b_3(2)); % triangle side 2
BT_2(:,3) = linspace(b_2(3),b_3(3));
BT_3(:,1) = linspace(b_3(1),b_1(1));
BT_3(:,2) = linspace(b_3(2),b_1(2)); % triangle side 3
BT_3(:,3) = linspace(b_3(3),b_1(3));
plot3(BT_1(:,1), BT_1(:,2), BT_1(:,3),'k');
hold on
plot3(BT_2(:,1), BT_2(:,2), BT_2(:,3),'k');
hold on
plot3(BT_3(:,1), BT_3(:,2), BT_3(:,3),'k');
% upper arms
UA_1(:,1) = linspace(B_1(1),A_1(1));
UA_1(:,2) = linspace(B_1(2),A_1(2)); % upper arm 1 components
UA_1(:,3) = linspace(B_1(3),A_1(3));
UA_2(:,1) = linspace(B_2(1),A_2(1));
UA_2(:,2) = linspace(B_2(2),A_2(2)); % upper arm 2
UA_2(:,3) = linspace(B_2(3),A_2(3));
UA_3(:,1) = linspace(B_3(1),A_3(1));
UA_3(:,2) = linspace(B_3(2),A_3(2)); % upper arm 3
UA_3(:,3) = linspace(B_3(3),A_3(3));
plot3(UA_1(:,1), UA_1(:,2), UA_1(:,3),'r');
hold on
plot3(UA_2(:,1), UA_2(:,2), UA_2(:,3),'g');
hold on
plot3(UA_3(:,1), UA_3(:,2), UA_3(:,3),'b');
% lower arms
LA_1(:,1) = linspace(A_1(1),Pf_1(1));
LA_1(:,2) = linspace(A_1(2),Pf_1(2)); % lower arm 1 components
LA_1(:,3) = linspace(A_1(3),Pf_1(3));
LA_2(:,1) = linspace(A_2(1),Pf_2(1));
LA_2(:,2) = linspace(A_2(2),Pf_2(2)); %lower arm 2
LA_2(:,3) = linspace(A_2(3),Pf_2(3));
LA_3(:,1) = linspace(A_3(1),Pf_3(1));
LA_3(:,2) = linspace(A_3(2),Pf_3(2)); %lower arm 3
LA_3(:,3) = linspace(A_3(3),Pf_3(3));
plot3(LA_1(:,1), LA_1(:,2), LA_1(:,3),'k');
hold on
plot3(LA_2(:,1), LA_2(:,2), LA_2(:,3),'k');
hold on
plot3(LA_3(:,1), LA_3(:,2), LA_3(:,3),'k');
% end-effector base
eet_1(:,1) = linspace(Pf_1(1),Pf_2(1));
eet_1(:,2) = linspace(Pf_1(2),Pf_2(2));
eet_1(:,3) = linspace(Pf_1(3),Pf_2(3));
eet_2(:,1) = linspace(Pf_2(1),Pf_3(1));
eet_2(:,2) = linspace(Pf_2(2),Pf_3(2)); %end effector
eet_2(:,3) = linspace(Pf_2(3),Pf_3(3));
eet_3(:,1) = linspace(Pf_3(1),Pf_1(1));
eet_3(:,2) = linspace(Pf_3(2),Pf_1(2));
eet_3(:,3) = linspace(Pf_3(3),Pf_1(3));
plot3(eet_1(:,1), eet_1(:,2), eet_1(:,3),'k');
hold on
plot3(eet_2(:,1), eet_2(:,2), eet_2(:,3),'k');
hold on
plot3(eet_3(:,1), eet_3(:,2), eet_3(:,3),'k');
hold on
grid on
title('Delta Robot Position');
xlabel('X');
ylabel('Y');
zlabel('Z');
plot3(P(1),P(2),P(3),'r*')