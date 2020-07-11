clear all;

%link lengths
L1=0.24765;
L2=0.2286;
L3=0.2286;
L5=0.073025;       

%Creating Links
%L=Link[( theta d a alpha)]
L(1)=Link([0     L1     0       -pi/2]  );
L(2)=Link([0     0      L2      0]      );
L(3)=Link([0     0      L3      0]      );
L(4)=Link([0     0      0       pi/2]   );
L(5)=Link([0     0      L5      0]      );

%Connecting Links serially
Rob=SerialLink(L);

%Transformation Matrix of the tool frame
XYZ=[0.146,0,0.409];
T=transl(XYZ);
RPY=[deg2rad(0),deg2rad(-90),deg2rad(-180)];
R=rpy2tr(RPY);
TR=T*R;

%Assigning the defined tool frame to robot tool frame
%Rob.tool=TR;
Rob %DH parameters

%Drawing the robot
Rob.plot([0 0 0 0 0]);

%Forward Kinematics
% Rob.fkine([0 0 0 0 0])

%Robot Animation
% syms theta1 theta2 theta3 theta4 theta5
% for theta1=0:0.1:pi/2
%     Rob.plot([theta1 0 0 0 0]);
%     pause(0.25)
% end
% 
% for theta=0:0.1:pi/2
%       Rob.plot([theta theta 0 0 0]);
%       pause(0.25)
% end
%     

