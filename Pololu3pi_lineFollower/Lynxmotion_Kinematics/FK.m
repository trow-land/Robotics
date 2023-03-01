%% Part 1.1

clear all;
clc;

syms c a alpha dN theta l1 l2 l3 q1 q2 q3 q4 q5 d;

Link = [1;2;3;4;5];
aN = [0;l1;l2;0;0];
alphaN = [90;0;0;90;0];
DN = [d;0;0;0;l3];
thetaN = [q1;q2;q3;q4;q5];

% Table of value calulated from the drawing of the Lynx Robot Arm
Distal_Table = table(Link, aN, alphaN, DN, thetaN)


%Indiviual transpose matrices from the distal table
 

T01 = [cos(q1) 0 sin(q1) 0; sin(q1) 0 cos(q1) 0; 0 1 0 d; 0 0 0 1];
T12 = [cos(q2) -sin(q2) 0 l1*cos(q2); sin(q2) cos(q2) 0 l1*sin(q2); 0 0 1 0; 0 0 0 1];
T23 = [cos(q3) -sin(q3) 0 l2*cos(q3); sin(q3) cos(q3) 0 l2*sin(q3); 0 0 1 0; 0 0 0 1];
T34 = [cos(q4) 0 sin(q4) 0; sin(q4) 0 -cos(q4) 0; 0 1 0 0; 0 0 0 1];
T45 = [cos(q5) -sin(q5) 0 0; sin(q5) cos(q5) 0 0; 0 0 1 l3; 0 0 0 1];

% Get the transpose from the origin to the end effector by multiplying all
% of the individual transposes together

T05 = T01*T12*T23*T34*T45;


%% -------------------------- Part 1.2 --------------------------------------------------


%% Links Lengths
l1 = 0.1 ;
l2 = 0.1 ;
l3 = 0.1 ;
d = 0.05;



%% Workspace
q1 = 0:pi/30:2*pi-pi/180 ;    %pi/180  = the step distance  <------------------ These look to be the ranges of motion for each joint.
q2 = 0:pi/10:pi-pi/180 ;
q3 = 0:pi/10:pi-pi/180 ;
q4 = 0:pi/10:pi-pi/180 ;
q5 = 0:pi/10:2*pi-pi/180 ;


%% Plot the workspace of the robot
figure (3)
set(3,'position',[1243 190 560 420])

Xwork = [];
Ywork = [];
Zwork = [];
% 


for i = 1:length(q1)	% for q1
    for j = 1:length(q2)   % for q2
        for k = 1:length(q3) % for q3
            for l = 1:length(q4) % for q4
                xwork = l3*(cos(q4(l))*(cos(q1(i))*cos(q2(j))*sin(q3(k)) + cos(q1(i))*cos(q3(k))*sin(q2(j))) + sin(q4(l))*(cos(q1(i))*cos(q2(j))*cos(q3(k)) - cos(q1(i))*sin(q2(j))*sin(q3(k)))) + l1*cos(q1(i))*cos(q2(j)) + l2*cos(q1(i))*cos(q2(j))*cos(q3(k)) - l2*cos(q1(i))*sin(q2(j))*sin(q3(k));
                ywork = l3*(cos(q4(l))*(cos(q2(j))*sin(q1(i))*sin(q3(k)) + cos(q3(k))*sin(q1(i))*sin(q2(j))) - sin(q4(l))*(sin(q1(i))*sin(q2(j))*sin(q3(k)) - cos(q2(j))*cos(q3(k))*sin(q1(i)))) + l1*cos(q2(j))*sin(q1(i)) + l2*cos(q2(j))*cos(q3(k))*sin(q1(i)) - l2*sin(q1(i))*sin(q2(j))*sin(q3(k));
                zwork = d + l1*sin(q2(j)) - l3*(cos(q4(l))*(cos(q2(j))*cos(q3(k)) - sin(q2(j))*sin(q3(k))) - sin(q4(l))*(cos(q2(j))*sin(q3(k)) + cos(q3(k))*sin(q2(j)))) + l2*cos(q2(j))*sin(q3(k)) + l2*cos(q3(k))*sin(q2(j));
                zwork = zwork - d;
                if zwork < 0
                    zwork = 0;
                end
                Xwork = [Xwork xwork];
                Ywork = [Ywork ywork];
                Zwork = [Zwork zwork];


            end
        end
    end  
end

figure(3)
plot3(Xwork,Ywork,Zwork, '.', 'color', 'b')
title('Lynxmotion 3D workspace')
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')
axis equal

figure(4)
plot(Xwork, Ywork,'.', 'color', 'b')
title('Lynxmotion 2D workspace')
xlabel('x-axis')
ylabel('y-axis')
axis equal



% %Inverse Kinematic test
% %q1 = 0:pi/30:2*pi-pi/180 ;    
% q1 = pi/4;
% %q2 = 0:pi/10:pi-pi/180 ;
% q2 = deg2rad(-21.326260);
% %q3 = 0:pi/10:pi-pi/180 ;
% q3 = deg2rad(107.399839);
% %q4 = 0:pi/10:pi-pi/180 ;
% q4 = deg2rad(-26.073578);
% q5 = 0;
% 
% Xwork = [];
% Ywork = [];
% Zwork = [];
% 
% for i = 1:length(q1)	% for q1
%     for j = 1:length(q2)   % for q2
%         for k = 1:length(q3) % for q3
%             for l = 1:length(q4) % for q4
%                 xwork = l3*(cos(q4(l))*(cos(q1(i))*cos(q2(j))*sin(q3(k)) + cos(q1(i))*cos(q3(k))*sin(q2(j))) + sin(q4(l))*(cos(q1(i))*cos(q2(j))*cos(q3(k)) - cos(q1(i))*sin(q2(j))*sin(q3(k)))) + l1*cos(q1(i))*cos(q2(j)) + l2*cos(q1(i))*cos(q2(j))*cos(q3(k)) - l2*cos(q1(i))*sin(q2(j))*sin(q3(k));
%                 ywork = l3*(cos(q4(l))*(cos(q2(j))*sin(q1(i))*sin(q3(k)) + cos(q3(k))*sin(q1(i))*sin(q2(j))) - sin(q4(l))*(sin(q1(i))*sin(q2(j))*sin(q3(k)) - cos(q2(j))*cos(q3(k))*sin(q1(i)))) + l1*cos(q2(j))*sin(q1(i)) + l2*cos(q2(j))*cos(q3(k))*sin(q1(i)) - l2*sin(q1(i))*sin(q2(j))*sin(q3(k));
%                 zwork = d + l1*sin(q2(j)) - l3*(cos(q4(l))*(cos(q2(j))*cos(q3(k)) - sin(q2(j))*sin(q3(k))) - sin(q4(l))*(cos(q2(j))*sin(q3(k)) + cos(q3(k))*sin(q2(j)))) + l2*cos(q2(j))*sin(q3(k)) + l2*cos(q3(k))*sin(q2(j));
%                 zwork = zwork - d;
%                 if zwork < 0
%                     zwork = 0;
%                 end
%                 Xwork = [Xwork xwork];
%                 Ywork = [Ywork ywork];
%                 Zwork = [Zwork zwork];
% 
% 
%             end
%         end
%     end  
% end
% 
% 
% figure(5)
% plot3(Xwork,Ywork,Zwork, '.', 'color', 'b')
% title('IK test')
% xlabel('x-axis')
% ylabel('y-axis')
% zlabel('z-axis')
% axis equal