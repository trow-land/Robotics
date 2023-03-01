
Corner1 = [-0.15, 0.15, 0.05];  % fine
Corner2 = [0.15, 0.15, 0.05];  % fine
Corner3 = [0.15, -0.15, 0.05];  %fine 
Corner4 = [-0.15, -0.15, 0.05];  % fine
Top = [0.00, 0.00, 0.25]; % fine


straightLine(Corner1, Corner2, Corner3, Corner4, Top);



function ikinematics(links1, links2, links3, positionx, positiony, positionz, gamma)
% Inputs:
% links1,links2,links3 are length of each links in the robotic arm
% base is the length of base of the robotic arm
% positionx,positiony are joint angles in reference to x-axis
% gamma is the orientation

format compact
format short

Corner1 = [-0.15, 0.15, 0.05];  % fine
Corner2 = [0.15, 0.15, 0.05];  % fine
Corner3 = [0.15, -0.15, 0.05];  %fine 
Corner4 = [-0.15, -0.15, 0.05];  % fine
Top = [0.00, 0.00, 0.25]; % fine



l1 = links1;
l2 = links2;
l3 = links3;
d1 = 0.05;
Xee = positionx;
Yee = positiony;
Zee = positionz;
g = gamma;

%position P4
x4 = Xee-(l3*cosd(g));
y4 = Yee-(l3*sind(g));
z4 = Zee - (d1*sind(g));
C = sqrt(x4^2 + y4^2);

if (l1+l2) > C
    %angle a and B
    a = acosd((l1^2 + l2^2 - C^2 )/(2*l1*l2));
    B = acosd((l1^2 + C^2 - l2^2 )/(2*l1*C));

    %joint angles elbow-down
    J2a = atan2d(y4,x4)-B;
    J3a = 180-a;
    J4a = g - J2a -J3a;

    %joint angles elbow-up
    J2b = atan2d(y4,x4)+B;
    J3b = -(180-a);
    J4b = g - J2b - J3b;

    fprintf('Elbow-DOWN: The joint 2, 3 and 4 angles are (%f,%f, %f).\n',J2a,J3a,J4a)
    fprintf('Elbow-UP: The joint 2, 3 and 4 angles are (%f,%f, %f).\n',J2b,J3b,J4b)
else 
    disp('     Dimension error!')
    disp('     End effecter is outside the workspace.')
    return
end

clf;
x2a = l1*cosd(J2a);
y2a = l1*sind(J2a);
z2a = l1*sind(g) + d1;

x2b = l1*cosd(J2b);
y2b = l1*sind(J2b);
z2b = l1*sind(g) + d1;


figure(1)
clf
r = l1 + l2 + l3;
daspect([1,1,1])
rectangle('Position',[-r,-r,2*r,2*r],'Curvature',[1,1],...
    'LineStyle',':')
line([0 x2a], [0 y2a],'Color','b')
line([x2a x4], [y2a y4],'Color','b')
line([x4 Xee], [y4 Yee],'Color','b')
line([0 x2b], [0 y2b],'Color','g','LineStyle','--')
line([x2b x4], [y2b y4],'Color','g','LineStyle','--')
line([x4 Xee], [y4 Yee],'Color','b','LineStyle','--')
 line([Corner1(1) Corner2(1)], [Corner1(2)  Corner2(2)], 'Color', 'r', 'LineStyle', '--')
 line([Corner2(1) Corner3(1)], [Corner2(2)  Corner3(2)], 'Color', 'r', 'LineStyle', '--')
 line([Corner3(1) Corner4(1)], [Corner3(2)  Corner4(2)], 'Color', 'r', 'LineStyle', '--')
 line([Corner4(1) Top(1)], [Corner4(2)  Top(2)], 'Color', 'r', 'LineStyle', '--')
%  plot(Corner1(1), Corner1(2), 'marker', 'x', 'markersize', 15, 'Color', 'k')
%  plot(Corner2(1), Corner2(2), 'marker', 'x', 'markersize', 15, 'Color', 'k')
%  plot(Corner3(1), Corner3(2), 'marker', 'x', 'markersize', 15, 'Color', 'k')
%  plot(Corner4(1), Corner4(2), 'marker', 'x', 'markersize', 15, 'Color', 'k')
%  plot(Top(1), Top(2), 'marker', 'x', 'markersize', 15, 'Color', 'k')
%line([0 xe], [0 ye],'Color','r')
line([0 0], [-r/10 r/10], 'Color', 'r')
line([-r/10 r/10], [0 0], 'Color', 'r')
hold on
plot([0 x2a x4],[0 y2a y4],'o','Color','b')
plot([x2b],[y2b],'o','Color','g')
plot([Xee],[Yee],'o', 'Color', 'r')
grid on
xlabel('x-axis')
ylabel('y-axis')
title('2D Plot of Straight Line Trajectory')











 % Plot the points
figure(3)
 clf
 hold on
 plot3(0, 0, d1, 'marker', '.', 'markersize', 10)
 plot3(Corner1(1), Corner1(2), Corner1(3), 'marker', 'x', 'markersize', 15, 'Color', 'k')
 plot3(Corner2(1), Corner2(2), Corner2(3), 'marker', 'x', 'markersize', 15, 'Color', 'k')
 plot3(Corner3(1), Corner3(2), Corner3(3), 'marker', 'x', 'markersize', 15, 'Color', 'k')
 plot3(Corner4(1), Corner4(2), Corner4(3), 'marker', 'x', 'markersize', 15, 'Color', 'k')
 plot3(Top(1), Top(2), Top(3), 'marker', 'x', 'markersize', 15, 'Color', 'k')
 line([Corner1(1) Corner2(1)], [Corner1(2)  Corner2(2)], [Corner1(3) Corner2(3)], 'Color', 'r', 'LineStyle', '--')
 line([Corner2(1) Corner3(1)], [Corner2(2)  Corner3(2)], [Corner2(3) Corner3(3)], 'Color', 'r', 'LineStyle', '--')
 line([Corner3(1) Corner4(1)], [Corner3(2)  Corner4(2)], [Corner3(3) Corner4(3)], 'Color', 'r', 'LineStyle', '--')
 line([Corner4(1) Top(1)], [Corner4(2)  Top(2)], [Corner4(3) Top(3)], 'Color', 'r', 'LineStyle', '--')
 line([0 0], [-r/10 r/10], [0 0], 'Color', 'r')
 line([-r/10 r/10], [0 0], [0 0], 'Color', 'r')
 line([0 0], [0 0], [-r/10 r/10], 'Color', 'r')
 axis([-0.3 0.3 -0.3 0.3 -0.3 0.3]);     % Axis dimensions
 % Option A path
 line([0 0], [0 0], [0 d1],'Color','b')
 line([0 x2a], [0 y2a], [d1 z2a],'Color','b')   % Origin to first joint q3
 line([x2a x4], [y2a y4], [z2a z4],'Color','b')   % q3 to q4
 line([x4 Xee], [y4 Yee ], [z4 Zee],'Color','b')  % q4 to ee
 % Option B path
 line([0 0], [0 0], [0 d1],'Color','b')
 line([0 x2b], [0 y2b], [d1 z2b],'Color','k','LineStyle','--')
 line([x2b x4], [y2b y4], [z2b z4],'Color','k','LineStyle','--')
 line([x4 Xee], [y4 Yee], [z4 Zee],'Color','b','LineStyle','--')
 plot3([Xee], [Yee], [Zee],'.', 'Color', 'r', 'markersize', 15) % end effector 
 plot3([0 x2a x4],[0 y2a y4],[0 z2a z4],'.','Color','b', 'markersize', 15)
 plot3([x2b],[y2b],[z2b],'.','Color','k', 'markersize', 15)
%  rectangle('Position',[-0.15,-0.15,0.3,0.3],'FaceColor','g');
x = [-0.25 0.25 0.25 -0.25];
y = [-0.25 -0.25 0.25 0.25];
z = [0 0 0 0];
fill3(x, y, z, 'g', 'FaceAlpha', 0.5)
 grid on
 hold off
            
 % Add labels and a title
 xlabel('x')
 ylabel('y')
 zlabel('z')
 title('3D Plot of Straight Line Trajectory')
            
 % Set the view
 view(-23, 38)


pause(2);
end


function straightLine(Corner1, Corner2, Corner3, Corner4, Top)


% Calculate the differences in x, y, and z between the corners.
delta12 = [(Corner2(1)-Corner1(1)), (Corner2(2)-Corner1(2)), (Corner2(3)-Corner1(3))];
delta23 = [(Corner3(1)-Corner2(1)), (Corner3(2)-Corner2(2)), (Corner3(3)-Corner2(3))];
delta34 = [(Corner4(1)-Corner3(1)), (Corner4(2)-Corner3(2)), (Corner4(3)-Corner3(3))];
delta45 = [(Top(1)-Corner4(1)), (Top(2)-Corner4(2)), (Top(3)-Corner4(3))];


disp(delta12);
disp(delta23);
disp(delta34);
disp(delta45);


% Divide the difference by the total no. of steps - Lets initially say 10

increment12 = delta12 / 10;
increment23 = delta23 / 10;
increment34 = delta34 / 10;
increment45 = delta45 / 10;


%Starting Positions
path12 = Corner1;
path23 = Corner2;
path34 = Corner3;
path45 = Corner4;

ikinematics(0.1,0.1,0.1, Corner1(1), Corner1(2), Corner1(3),140);

for n = 0:10
    path12 = path12 + increment12;
    ikinematics(0.1,0.1,0.1, path12(1), path12(2), path12(3),100);
end


%ikinematics(0.1,0.1,0.1, Corner2(1), Corner2(2), Corner2(3),140);

for n = 0:10
    path23 = path23 + increment23;
    ikinematics(0.1,0.1,0.1, path23(1), path23(2), path23(3),20);
end
% pause(1);
% ikinematics(0.1,0.1,0.1, Corner3(1), Corner3(2), Corner3(3),20);
% disp("1");
% pause(1);
% ikinematics(0.1,0.1,0.1, Corner3(1), Corner3(2), Corner3(3),30);
% disp("2");
% pause(1);
% ikinematics(0.1,0.1,0.1, Corner3(1), Corner3(2), Corner3(3),45);
% disp("3");
% pause(1);
% ikinematics(0.1,0.1,0.1, Corner3(1), Corner3(2), Corner3(3),120);
% disp("4");
% pause(1);
% ikinematics(0.1,0.1,0.1, Corner3(1), Corner3(2), Corner3(3),50);
% disp("5");
% pause(1);

for n = 0:3
    path34 = path34 + increment34;
    ikinematics(0.1,0.1,0.1, path34(1), path34(2), path34(3),20);
    pause(0.5);
end

for n = 0:2
    path34 = path34 + increment34;
    ikinematics(0.1,0.1,0.1, path34(1), path34(2), path34(3),0);
    pause(0.5);
end

for n = 0:5
    path34 = path34 + increment34;
    ikinematics(0.1,0.1,0.1, path34(1), path34(2), path34(3),160);
    pause(0.5);
end



for n = 0:9
    path45 = path45 + increment45;
    ikinematics(0.1,0.1,0.1, path45(1), path45(2), path45(3),120);
    pause(0.5);
end


end