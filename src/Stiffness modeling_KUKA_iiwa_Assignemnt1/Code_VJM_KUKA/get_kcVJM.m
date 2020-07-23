function Kc = get_kcVJM(x, y, z)
%%%%%%  Homework 1%%%%%%
%%%%%% Victor Massague , Sami SELLAMI %%%%%%%
%%%%%% function returns the VJM stiffness matrix Kc of the KUKA iiwa manipulator angles %%%%%%%%%%
%%%%%% input: positions x, y, z %%%%%%%%%%%%%%%%%
%%%%%% output: Matrix Kc   %%%%%%%%%%%%%%%%%%%%%%%%%%%%

ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
iiwa=KST(ip,arg1,arg2,Tef_flange);
Tt = eye(4);
%Tt(1:3,1:3) = -Tt(1:3,1:3);
Tt(1:3,4) = [x;y;z];
q = iiwa.gen_InverseKinematics(zeros(7,1), Tt,1000,0.5);

if checkLimits(q)
    Tbase = eye(4);
    Ttool = eye(4);

    % Actuator stiff 1
    k0 = 1e6; 

    %LINK1
    %material and shape parameters
    E = 70 *10e9; %Young's modulus
    G = 25.5*10e9; %shear modulus
    robot.d(1) = 10*10e-3;
    robot.L(1) = 0.21;

    %for cylinder
    S = pi*robot.d(1)^2/4;
    Iy = pi*robot.d(1)^4/64;
    Iz = pi*robot.d(1)^4/64;
    J = Iy + Iz;


    k1 = [E*S/robot.L(1) 0                  0                 0           0                 0;
        0           12*E*Iz/robot.L(1)^3  0                 0           0                 6*E*Iy/robot.L(1)^2;
        0           0                  12*E*Iy/robot.L(1)^3 0           -6*E*Iy/robot.L(1)^2 0;
        0           0                  0                 G*J/robot.L(1) 0                 0;
        0           0                  -6*E*Iy/robot.L(1)^2 0           4*E*Iy/robot.L(1)      0;
        0           6*E*Iy/robot.L(1)^2   0                 0           0                 4*E*Iz/robot.L(1)];


    % Actuator stiff 2
    k2 = 1e6; 

    %LINK2
    %material and shape parameters
    robot.d(2) = 10*10e-3;
    robot.L(2) = 0.21;


    k3 = [E*S/robot.L(2) 0                  0                 0           0                 0;
        0           12*E*Iz/robot.L(2)^3  0                 0           0                 6*E*Iy/robot.L(2)^2;
        0           0                  12*E*Iy/robot.L(2)^3 0           -6*E*Iy/robot.L(2)^2 0;
        0           0                  0                 G*J/robot.L(2) 0                 0;
        0           0                  -6*E*Iy/robot.L(2)^2 0           4*E*Iy/robot.L(2)      0;
        0           6*E*Iy/robot.L(2)^2   0                 0           0                 4*E*Iz/robot.L(2)];


    % Actuator stiff 3
    k4 = 1e6; 

    %LINK3
    %material and shape parameters
    robot.d(3) = 10*10e-3;
    robot.L(3) = 0.21;

    k5 = [E*S/robot.L(3) 0                  0                 0           0                 0;
        0           12*E*Iz/robot.L(3)^3  0                 0           0                 6*E*Iy/robot.L(3)^2;
        0           0                  12*E*Iy/robot.L(3)^3 0           -6*E*Iy/robot.L(3)^2 0;
        0           0                  0                 G*J/robot.L(3) 0                 0;
        0           0                  -6*E*Iy/robot.L(3)^2 0           4*E*Iy/robot.L(3)      0;
        0           6*E*Iy/robot.L(3)^2   0                 0           0                 4*E*Iz/robot.L(3)];

    % Actuator stiff 4
    k6 = 1e6; 

    %LINK4
    %material and shape parameters
    robot.d(4) = 10*10e-3;
    robot.L(4) = 0.2;

    k7 = [E*S/robot.L(4) 0                  0                 0           0                 0;
        0           12*E*Iz/robot.L(4)^3  0                 0           0                 6*E*Iy/robot.L(4)^2;
        0           0                  12*E*Iy/robot.L(4)^3 0           -6*E*Iy/robot.L(4)^2 0;
        0           0                  0                 G*J/robot.L(4) 0                 0;
        0           0                  -6*E*Iy/robot.L(4)^2 0           4*E*Iy/robot.L(4)      0;
        0           6*E*Iy/robot.L(4)^2   0                 0           0                 4*E*Iz/robot.L(4)];

    % Actuator stiff 5
    k8 = 1e6; 

    %LINK5
    %material and shape parameters
    robot.d(5) = 10*10e-3;
    robot.L(5) = 0.2;

    k9 = [E*S/robot.L(5) 0                  0                 0           0                 0;
        0           12*E*Iz/robot.L(5)^3  0                 0           0                 6*E*Iy/robot.L(5)^2;
        0           0                  12*E*Iy/robot.L(5)^3 0           -6*E*Iy/robot.L(5)^2 0;
        0           0                  0                 G*J/robot.L(5) 0                 0;
        0           0                  -6*E*Iy/robot.L(5)^2 0           4*E*Iy/robot.L(5)      0;
        0           6*E*Iy/robot.L(5)^2   0                 0           0                 4*E*Iz/robot.L(5)];

    % Actuator stiff 6
    k10 = 1e6; 

    %LINK6
    %material and shape parameters
    robot.d(6) = 10*10e-3;
    robot.L(6) = 0.126;

    k11 = [E*S/robot.L(6) 0                  0                 0           0                 0;
        0           12*E*Iz/robot.L(6)^3  0                 0           0                 6*E*Iy/robot.L(6)^2;
        0           0                  12*E*Iy/robot.L(6)^3 0           -6*E*Iy/robot.L(6)^2 0;
        0           0                  0                 G*J/robot.L(6) 0                 0;
        0           0                  -6*E*Iy/robot.L(6)^2 0           4*E*Iy/robot.L(6)      0;
        0           6*E*Iy/robot.L(6)^2   0                 0           0                 4*E*Iz/robot.L(6)];

    % Actuator stiff 7
    k12 = 1e6; 


    %LINK7 (tool)
    %material and shape parameters

    robot.d(7) = 10*10e-3;
    robot.L(7) = 0.05;
    S = pi*robot.d(7)^2/4;

    k13 = [E*S/robot.L(7) 0                  0                 0           0                 0;
    0           12*E*Iz/robot.L(7)^3  0                 0           0                 6*E*Iy/robot.L(7)^2;
    0           0                  12*E*Iy/robot.L(7)^3 0           -6*E*Iy/robot.L(7)^2 0;
    0           0                  0                 G*J/robot.L(7) 0                 0;
    0           0                  -6*E*Iy/robot.L(7)^2 0           4*E*Iy/robot.L(7)      0;
    0           6*E*Iy/robot.L(7)^2   0                 0           0                 4*E*Iz/robot.L(7)];


    % K theta matrixshow
    Kt = [k0 zeros(1,42) zeros(1,6)
        zeros(6,1) k1 zeros(6,36) zeros(6,6)
        zeros(1,1) zeros(1,6) k2 zeros(1,35) zeros(1,6)
        zeros(6,1) zeros(6,6) zeros(6,1) k3 zeros(6,29) zeros(6,6)
        zeros(1,1) zeros(1,6) zeros(1,1) zeros(1,6) k4 zeros(1,28) zeros(1,6)
        zeros(6,1) zeros(6,6) zeros(6,1) zeros(6,6) zeros(6,1) k5 zeros(6,22) zeros(6,6)
        zeros(1,1) zeros(1,6) zeros(1,1) zeros(1,6) zeros(1,1) zeros(1,6) k6 zeros(1,21) zeros(1,6)
        zeros(6,1) zeros(6,6) zeros(6,1) zeros(6,6) zeros(6,1) zeros(6,6) zeros(6,1) k7 zeros(6,15) zeros(6,6)
        zeros(1,1) zeros(1,6) zeros(1,1) zeros(1,6) zeros(1,1) zeros(1,6) zeros(1,1) zeros(1,6)  k8 zeros(1,14) zeros(1,6)
        zeros(6,1) zeros(6,6) zeros(6,1) zeros(6,6) zeros(6,1) zeros(6,6) zeros(6,1) zeros(6,6) zeros(6,1) k9 zeros(6,8) zeros(6,6)
        zeros(1,1) zeros(1,6) zeros(1,1) zeros(1,6) zeros(1,1) zeros(1,6) zeros(1,1) zeros(1,6) zeros(1,1) zeros(1,6) k10 zeros(1,7) zeros(1,6)
        zeros(6,1) zeros(6,6) zeros(6,1) zeros(6,6) zeros(6,1) zeros(6,6) zeros(6,1) zeros(6,6) zeros(6,1) zeros(6,6) zeros(6,1) k11 zeros(6,1) zeros(6,6)
        zeros(1,1) zeros(1,6) zeros(1,1) zeros(1,6) zeros(1,1) zeros(1,6) zeros(1,1) zeros(1,6) zeros(1,1) zeros(1,6) zeros(1,1) zeros(1,6) k12 zeros(1,6)
        zeros(6,1) zeros(6,6) zeros(6,1) zeros(6,6) zeros(6,1) zeros(6,6) zeros(6,1) zeros(6,6) zeros(6,1) zeros(6,6) zeros(6,1) zeros(6,6) zeros(6,1) k13 ];
    
    t= zeros(1, 49);
    d = [robot.L(1), robot.L(2), robot.L(3), robot.L(4), robot.L(5), robot.L(6), robot.L(7)];
    Jth = Jt(Tbase,Ttool,q,t,d);
    Kc = inv(inv(Jth*inv(Kt)*Jth'));
else
    Kc = 0;
end
end