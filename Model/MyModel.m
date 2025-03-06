%Clear the variables from the workspace
clear;
%% Create symbolic variables for states, inputs, MVs and parameters
syms input_1 input_2 input_3 input_4 Ixx Iyy Izz k l m b g
syms x y z phi theta psi u v w p q r
% g: gravitational acceleration [m/s^2]
% b: rotor drag constant [kg*m^2]
% k: rotor lift constant [kg*m]
% l: moment arm between rotor and centre of mass [m]
% (Iii): diagonal elements of inertia matrix [kg*m^2]

load UAV_NominalParameters % Make sure this is in your path
%IxxVal  %Units kg * m^2
%IyyVal  %Units kg * m^2
%IzzVal  %Units kg * m^2
%kVal  %Units N * s^2 = kg * m
%lVal  %Units m
%mVal  %Units kg
%bVal  %Units N * m * s^2= kg * m^2
%omegaMax2Val  %Units (rad / s)^2
gVal = 9.81; %Units kg * m / s^2

paramValues = [IxxVal IyyVal IzzVal kVal lVal mVal bVal gVal omegaMax2Val];
%R-ZYX Euler
Rz = [cos(psi),     -sin(psi), 0;
      sin(psi),     cos(psi),  0;
      0,            0,         1];

Ry = [cos(theta),   0,         sin(theta);
      0,            1,         0;
      -sin(theta),  0,         cos(theta)];

Rx = [1,            0,         0;
      0,            cos(phi),  -sin(phi);
      0,            sin(phi),  cos(phi)];

% Rotation matrix from body frame to inertial frame
R_Euler = Rz*Ry*Rx;

% Thrust force
Fz = - k * (input_1 + input_2 + input_3 + input_4)*omegaMax2Val;


% Moments
L = k * l * (input_1 - input_2 - input_3 + input_4) * omegaMax2Val;
M = k * l * (input_1 + input_2 - input_3 - input_4) * omegaMax2Val;
% L and M are moments acting on the x and y-axis respectively and l is the moment arm of the thrust
N = b * (-input_1 + input_2 - input_3 + input_4) * omegaMax2Val;


%% deriving equations of motion
% Position in global frame of reference
f(1:3) = [u; v; w];
% Euler angles rates
f(4) = p + (q * sin(phi) + r * cos(phi)) * tan(theta);
f(5) = q * cos(phi) - r * sin(phi);
f(6) = (q * sin(phi) + r * cos(phi)) / cos(theta);
% Acceleration in inertial frame
f(7:9) = R_Euler*[0;0;Fz]/m + [0;0;g];
% Rotational Acceleration
f(10) = (L + (Iyy - Izz) * q * r) / Ixx;
f(11) = (M + (Izz - Ixx) * p * r) / Iyy;
f(12) = (N + (Ixx - Iyy) * p * q) / Izz;

f = subs(f, [Ixx Iyy Izz k l m b g omegaMax2Val], paramValues); 
% paramValues = [IxxVal IyyVal IzzVal kVal lVal mVal bVal gVal omegaMax2Val]
f = simplify(f); 
% Group symbolic variables
state = [x y z phi theta psi u v w p q r];
control = [input_1, input_2, input_3, input_4];

% Calculate linearization
A = jacobian(f,state);
B = jacobian(f,control);

disp('--------------------------------------------------------------------------------------------------------------------------')
% A = subs(A,[phi,theta,psi, p, q, r, input_1,input_2,input_3,input_4],[0,0,0, 0, 0, 0, 0.473016586187337,0.473016586187337,0.473016586187337,0.473016586187337]);
% B = subs(B,[phi,theta,psi,input_1,input_2,input_3,input_4],[0,0,0,0.473016586187337,0.473016586187337,0.473016586187337,0.473016586187337]);
% A = double(A);
% B = double(B);
disp(A)
disp(B)
disp('--------------------------------------------------------------------------------------------------------------------------')

disp('--------------------------------------------------------------------------------------------------------------------------')

disp('--------------------------------------------------------------------------------------------------------------------------')

disp('--------------------------------------------------------------------------------------------------------------------------')




%% save the obtained state dynamics function
% Create appropriate files
% Create QuadrotorStateFcn.m
matlabFunction(transpose(f),'File','QuadrotorStateFcnBase',...
    'Vars',{transpose(state),transpose(control)});
% Create QuadrotorStateJacobianFcn.m 
matlabFunction(A, B,'File','QuadrotorStateJacobianFcnBase',...
    'Vars',{transpose(state),transpose(control)});
f_StatesZero = subs(f, state, zeros(1,12));                       
[input_Eq1, input_Eq2, input_Eq3, input_Eq4] = solve(f_StatesZero == 0, control);     
input_Eq = double([input_Eq1 input_Eq2 input_Eq3 input_Eq4]);

%Save the result in a mat file
save('Equilibrium inputs', 'input_Eq')

in1 = [0 0 0 0 0 0 0 0 0 0 0 0]';
in2 = [0.473016586187337,	0.473016586187337,	0.473016586187337,	0.473016586187337]';

[A,B] = QuadrotorStateJacobianFcnBase(in1,in2);