function param = mySetup(shape)
    %%%
    
    % 
    % shape.constraints.ellipses{1} = ellipse;
    % shape.eps_t       = 0.02;
    % shape.start       = start;
    % shape.target      = target;
    % shape.Tt          = Tt;
    % shape.Te          = Te;
    % shape.ThetaMax    = ThetaMax;
    %%%
    param.Ts = 0.14;     % This is a sample way to set your sampling time
    N = 10; % Prediction horizon
    s0 = [shape.start(1), shape.start(2), shape.start(3), 0, 0, 0, 0, 0, 0, 0, 0, 0]';
    x_target = [shape.target(1), shape.target(2), shape.target(3), 0, 0, 0, 0, 0, 0, 0, 0, 0]';
    u_eq = [0.473016586187337	0.473016586187337	0.473016586187337	0.473016586187337]';

    Q = diag([6, 6, 6, 0.1, 0.1, 10e-6, 1, 1, 1, 1, 1, 1]);
    R = diag([0.01, 0.01, 0.01, 0.01]);
    P = Q;

    % linear constraints: The states box and inputs boundaries
    xa = shape.constraints.rect.bot(1,1);
    ya = shape.constraints.rect.bot(1,2);
    xb = shape.constraints.rect.bot(2,1);
    yb = shape.constraints.rect.bot(2,2);
    xc = shape.constraints.rect.bot(3,1);
    yc = shape.constraints.rect.bot(3,2);
    xd = shape.constraints.rect.bot(4,1);
    yd = shape.constraints.rect.bot(4,2);
    hmax = -1*shape.constraints.rect.h;
     
    % StageConstraints  D*s_k+1 + E*q_k <= b
    D = zeros(14, 12);
    D(1:6,:) = [ ya-yb  xb-xa  0    0 0 0 0 0 0 0 0 0;
                 yb-yc  xc-xb  0    0 0 0 0 0 0 0 0 0;
                 yc-yd  xd-xc  0    0 0 0 0 0 0 0 0 0;
                 yd-ya  xa-xd  0    0 0 0 0 0 0 0 0 0;
                 0      0      1    0 0 0 0 0 0 0 0 0;
                 0      0     -1    0 0 0 0 0 0 0 0 0;];
    
    E = [  0 0 0 0; 
            0 0 0 0;
            0 0 0 0; 
            0 0 0 0;
            0 0 0 0; 
            0 0 0 0;
            1  0  0  0; 
            0  1  0  0;
            0  0  1  0; 
            0  0  0  1;
           -1  0  0  0; 
            0 -1  0  0;
            0  0 -1  0; 
            0  0  0 -1;];
 
    b = [-xa*yb+xb*ya -xb*yc+xc*yb -xc*yd+xd*yc -xd*ya+xa*yd 0 hmax 1-0.4730 1-0.4730 1-0.4730 1-0.4730 0.4730 0.4730 0.4730 0.4730]';
    
    q = sym('q', [4, N]); 

    s = sym('s', [12, N]);

    



    












end % End of mySetup
function u = myMPController(param, y)
%% Design your controller here
    u = zeros(4,1);      % Create the output array of the appropriate size    
end % End of myMPController