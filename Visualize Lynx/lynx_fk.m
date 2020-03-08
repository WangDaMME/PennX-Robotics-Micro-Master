function [ pos ] = lynx_fk( theta1, theta2, theta3, theta4, theta5, g )
%LYNX_FK The input to the function will be the joint
%    angles of the robot in radians, and the distance between the gripper pads in inches.
%    The output must contain 10 positions of various points along the robot arm as specified
%    in the question.

    %% YOUR CODE GOES HERE
    %% 5 dof, 5 joint angles theta1-theta5
    %% DH Parameters
    %%Joint     a       alpha       d         theta 
    %%  1       0        -90       a=3in     theta1      
    %%  2       b=5.75    0          0       theta2-90   
    %%  3       c=7.375   0          0       theta3+90   
    %%  4       0        -90         0       theta4-90
    %%  5       0         0       d=4.125    theta5   
    pos = zeros(10, 3);
    
    A_01=compute_dh_matrix(0,    -pi/2,    3,     theta1);
    A_12=compute_dh_matrix(5.75,   0,      0,     theta2-pi/2);
    A_23=compute_dh_matrix(7.375,  0,      0,     theta3+pi/2);
    A_34=compute_dh_matrix(0,    -pi/2,    0,     theta4-pi/2);
    A_45=compute_dh_matrix(0,    0,      4.125,    theta5);
    
    T_01=A_01
    T_02=A_01*A_12
    T_03=A_01*A_12*A_23
    T_04=A_01*A_12*A_23*A_34
    T_05=A_01*A_12*A_23*A_34*A_45
    
    %% world frame
    world=[0 ; 0 ; 0];
    pos(2,:)=T_01(1:3,4)
    pos(3,:)=T_02(1:3,4)
    pos(4,:)=T_03(1:3,4)
    pos(5,:)=T_04(1:3,4)

    gripper_1= T_05 * [0;0; -1.125; 1];    
    gripper_2 = T_05 * [g/2;0; -1.125; 1];
    gripper_3 = T_05 * [-g/2;0; -1.125; 1];
    gripper_4= T_05 * [g/2;0; 0; 1];
    gripper_5= T_05 * [-g/2;0; 0; 1];   
    
    pos(6,:)=gripper_1(1:3);
    pos(7,:)=gripper_2(1:3);
    pos(8,:)=gripper_3(1:3);
    pos(9,:)=gripper_4(1:3);
    pos(10,:)=gripper_5(1:3);
end

function A = compute_dh_matrix(r, alpha, d, theta)

    %% Your code goes here
    A = eye(4);
    A=[cos(theta)   -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)  r*cos(theta);
   sin(theta)    cos(theta)*cos(alpha)   -cos(theta)*sin(alpha)     r*sin(theta);
   0    sin(alpha)   cos(alpha)      d
   0                        0                            0          1           ];
    
end