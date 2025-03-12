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
    
    param.Ts = 0.08;     % This is a sample way to set your sampling time
    N = 10; % Prediction horizon
    param.N = N;
    % param.s0 = [shape.start(1), shape.start(2), shape.start(3), 0, 0, 0, 0, 0, 0, 0, 0, 0]';
    param.x_target = [shape.target(1), shape.target(2), shape.target(3), 0, 0, 0, 0, 0, 0, 0, 0, 0]';
    param.u_eq = [0.473016586187337	0.473016586187337	0.473016586187337	0.473016586187337]';

    param.Q = diag([12, 12, 12, 0.1, 0.1, 10e-6, 1, 1, 1, 1, 1, 1]);
    param.R = diag([0.01, 0.01, 0.01, 0.01]);
    param.P = diag([15, 15, 15, 0.1, 0.1, 10e-6, 1, 1, 1, 1, 1, 1]);

    % linear constraints: The states box and inputs boundaries
    param.xa = shape.constraints.rect.bot(1,1);
    param.ya = shape.constraints.rect.bot(1,2);
    param.xb = shape.constraints.rect.bot(2,1);
    param.yb = shape.constraints.rect.bot(2,2);
    param.xc = shape.constraints.rect.bot(3,1);
    param.yc = shape.constraints.rect.bot(3,2);
    param.xd = shape.constraints.rect.bot(4,1);
    param.yd = shape.constraints.rect.bot(4,2);
    param.hmax = -1*shape.constraints.rect.h;
 
    param.ellipses = shape.constraints.ellipses;
    param.NumberofEllipses = size(param.ellipses, 2);
     


end % End of mySetup
