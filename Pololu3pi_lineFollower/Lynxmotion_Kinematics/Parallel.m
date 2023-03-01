clear all;
clc;
clf;


%  Defined robot dimensions
Sa=170;
L=130;
Rplatform = 130;
Rbase =290;



% Define the base positions
xbase1 = 0;   % Bottom left
ybase1 = 0;

xbase2 = Rbase*sqrt(3);  % Bottom Right
ybase2 = 0;

xbase3 = xbase2/2;            % Top 
ybase3 = xbase2*sind(60);

% X and Y end effector positions
x_centre = [180 360];
y_centre = [200 100];


% Platform orientation
alpha = [-10 0];  


for n = 1:2    % 2 as specified in question

    a = deg2rad(alpha(n)); 



% angle between ee-base point & x axis
psi1= a + (pi/6);
psi2 = a +(5*pi/6);
psi3 = (a - (pi/2));


xplat1 = x_centre(n) -Rplatform*cos(psi1);  % btm left
yplat1 = y_centre(n) -Rplatform*sin(psi1);

xplat2 = x_centre(n) -Rplatform*cos(psi2); % btm right
yplat2 = y_centre(n) -Rplatform*sin(psi2);

xplat3 = x_centre(n) -Rplatform*cos(psi3); % Top
yplat3 = y_centre(n) -Rplatform*sin(psi3);

% Link1 - Bottom Left

%Difference in x and y postions between plat and base
 delta_y1 = yplat1 - ybase1;
 delta_x1 = xplat1 - xbase1;

 angle_base_ee1 = atan2(delta_y1,delta_x1);

 %Cosine law on inner triangle
 angle_ee_joint1 = ((Sa^2 - L^2+delta_x1^2+delta_y1^2)/(2*Sa*sqrt(delta_x1^2+delta_y1^2)));
 angle_ee_joint1 = acos(angle_ee_joint1);

 q11= angle_base_ee1+angle_ee_joint1;
 q12 = angle_base_ee1-angle_ee_joint1;
 q11 = rad2deg(q11);
 q12 = rad2deg(q12);

 % Link2 - Bottom Right
 delta_y2 = yplat2-ybase2;
 delta_x2 = xplat2 -xbase2;

 angle_base_ee2 = atan2(delta_y2,delta_x2);
 angle_ee_joint2 = ((Sa^2 - L^2+delta_x2^2+delta_y2^2)/(2*Sa*sqrt(delta_x2^2+delta_y2^2)));
 angle_ee_joint2 = acos(angle_ee_joint2);

 q21= angle_base_ee2+angle_ee_joint2;
 q22 = angle_base_ee2-angle_ee_joint2;
 q21 = rad2deg(q21);
 q22 = rad2deg(q22);


 % Link 3 - Top
 delta_y3 = ybase3-yplat3;
 delta_x3 = xbase3 - xplat3;

 angle_base_ee3 = atan2(delta_y3,delta_x3) -pi;
 angle_ee_joint3 = ((Sa^2 - L^2+delta_x3^2+delta_y3^2)/(2*Sa*sqrt(delta_x3^2+delta_y3^2)));
 angle_ee_joint3 = acos(angle_ee_joint3);

 q31 = (angle_base_ee3+angle_ee_joint3);
 q32 = (angle_base_ee3-angle_ee_joint3);
 q31 = rad2deg(q31);
 q32 = rad2deg(q32);

 %Draw Links
 x_joint_11 = xbase1 + Sa*cosd(q11);
 y_joint_11 = ybase1 + Sa*sind(q11);
 x_joint_21 = xbase2 + Sa*cosd(q21);
 y_joint_21 = ybase2 + Sa*sind(q21);
 x_joint_31 = xbase3 + Sa*cosd(q31);
 y_joint_31 = ybase3 + Sa*sind(q31);


 %Draw Platform Triangle
 line([xplat1 xplat2], [yplat1 yplat2],'Color','r')
 line([xplat2 xplat3], [yplat2 yplat3],'Color','r')
 line([xplat3 xplat1], [yplat3 yplat1],'Color','r')
 hold on

 %Draw Base Triangle
 line([xbase1 xbase2], [ybase1 ybase2],'Color','k')
 line([xbase2 xbase3], [ybase2 ybase3],'Color','k')
 line([xbase3 xbase1], [ybase3 ybase1],'Color','k')

%Draw Links
 line([xbase1 x_joint_11], [ybase1 y_joint_11],'Color','b')
 line([x_joint_11 xplat1], [y_joint_11 yplat1],'Color','b')
 line([xplat1 x_centre(n)], [yplat1 y_centre(n)],'Color','b')


 line([xbase2 x_joint_21], [ybase2 y_joint_21],'Color','b')
 line([x_joint_21 xplat2], [y_joint_21 yplat2],'Color','b')
 line([xplat2 x_centre(n)], [yplat2 y_centre(n)],'Color','b')


 line([xbase3 x_joint_31], [ybase3 y_joint_31],'Color','b')
 line([x_joint_31 xplat3], [y_joint_31 yplat3],'Color','b')
 line([xplat3 x_centre(n)], [yplat3 y_centre(n)],'Color','b')


 plot([x_joint_11 x_joint_21 x_joint_31], [y_joint_11 y_joint_21 y_joint_31],'.','Color','b', MarkerSize= 15);
 plot([x_centre(n)], [y_centre(n)],'o','Color','k', MarkerSize= 5);


  % Add labels and a title
    xlabel('x')
    ylabel('y')

 title('Parallel Robot Plot')
            

 end
