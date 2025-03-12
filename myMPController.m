

function u = myMPController(param, y)
    
    N = param.N;
    Ts = param.Ts;
    Q = param.Q;
    R = param.R;
    P = param.P;
    x_target = param.x_target;
    xa = param.xa;
    ya = param.ya;
    xb = param.xb;
    yb = param.yb;
    xc = param.xc;
    yc = param.yc;
    xd = param.xd;
    yd = param.yd;
    u_eq = param.u_eq;
    hmax = param.hmax;
    % ellipse.a
    % ellipse.b 
    % ellipse.h  
    % ellipse.xc 
    % ellipse.yc 
    ellipses = param.ellipses;
    NumberofEllipses = param.NumberofEllipses;
    


        
   
     % NonlinearConstraints function
    function [c ceq] = mynlconstraints(U)
        c = [];
        ceq = [];
        xk = y;

        % beta = 60;
        
        % from x1 to xN
        for k = 1:1:N
            uk = U(4*k-3:4*k)';
            xdot = QuadrotorStateFcnBase(xk, uk);
            xk1 = xk + 0.25 * Ts * xdot;
            xk2 = xk + 0.5 * Ts * xdot;
            xk3 = xk + 0.75 * Ts * xdot;
            xk = xk + Ts * xdot;
            

            % constraints on position
            

            ck1 = (ya-yb) * xk(1) + (xb-xa) * xk(2) + xa*yb - xb*ya;
            ck2 = (yb-yc) * xk(1) + (xc-xb) * xk(2) + xb*yc - xc*yb;
            ck3 = (yc-yd) * xk(1) + (xd-xc) * xk(2) + xc*yd - xd*yc;
            ck4 = (yd-ya) * xk(1) + (xa-xd) * xk(2) + xd*ya - xa*yd;
            ck5 = xk(3);
            ck6 = -xk(3) - hmax;
            c  = [c, ck1 , ck2, ck3, ck4, ck5, ck6];
            % % Ellipses constraints
            for i = 1:1:NumberofEllipses
                ella = ellipses{i}.a;
                ellb = ellipses{i}.b;
                ellh = ellipses{i}.h;  
                ellxc = ellipses{i}.xc; 
                ellyc = ellipses{i}.yc; 
                % out the ellipse
                c_in_ell = - ((xk(1) - ellxc)^2)/(ella+0.001)^2 - ((xk(2) - ellyc)^2)/(ellb+0.001)^2 +1;
                c_h_ell = - (-0.001 + ellh) + xk(3);


                c_in_ell1 = - ((xk1(1) - ellxc)^2)/(ella+0.001)^2 - ((xk1(2) - ellyc)^2)/(ellb+0.001)^2 +1;
                c_h_ell1 = - (-0.001 + ellh) + xk1(3);

                c_in_ell2 = - ((xk2(1) - ellxc)^2)/(ella+0.001)^2 - ((xk2(2) - ellyc)^2)/(ellb+0.001)^2 +1;
                c_h_ell2 = - (-0.001 + ellh) + xk2(3);

                c_in_ell3 = - ((xk3(1) - ellxc)^2)/(ella+0.001)^2 - ((xk3(2) - ellyc)^2)/(ellb+0.001)^2 +1;
                c_h_ell3 = - (-0.001 + ellh) + xk3(3);



                
                % softmin
                % c_ell = - (1 / beta) * log(exp(-beta * c_in_ell) + exp(-beta * c_h_ell));
                % c_ell1 = - (1 / beta) * log(exp(-beta * c_in_ell1) + exp(-beta * c_h_ell1));
                % c_ell2 = - (1 / beta) * log(exp(-beta * c_in_ell2) + exp(-beta * c_h_ell2));
                % c_ell3 = - (1 / beta) * log(exp(-beta * c_in_ell3) + exp(-beta * c_h_ell3));

                c_ell = min([c_in_ell c_h_ell ]);
                c_ell1 = min([c_in_ell1 c_h_ell1 ]);
                c_ell2 = min([c_in_ell2 c_h_ell2 ]);
                c_ell3 = min([c_in_ell3 c_h_ell3 ]);

                c = [c, c_ell c_ell1 c_ell2 c_ell3];
            end
        end

         c = max(c);

    end

    param.mynlconstraints = @mynlconstraints;


    % Cost function
    function f = mycostfun(U)
        f = 0;
        xk = y;
        
        mui = 0.00;

        for k = 1:1:N-1
            uk = U(4*k-3 : 4*k)';
            xdot = QuadrotorStateFcnBase(xk, uk);

            
            xk = xk + (Ts) * xdot;
            
            f  = f + (xk - x_target)' * Q * (xk - x_target) + (uk - u_eq)' * R * (uk - u_eq);

            

            for i = 1:1:NumberofEllipses
                ella = ellipses{i}.a;
                ellb = ellipses{i}.b;
                ellh = ellipses{i}.h;  
                ellxc = ellipses{i}.xc; 
                ellyc = ellipses{i}.yc; 
                % out the ellipse
                % add soft constraint
                softc1 = - ((xk(1) - ellxc)^2)/(ella + 0.01)^2 - ((xk(2) - ellyc)^2)/(ellb + 0.01)^2 +1;
                softc2 = - (0.01 + ellh) + xk(3);
                f = f + mui * (min([softc1 softc2]));
            end
            
            

        end

        uk = U(4 * N - 3: 4 * N)'; % Note that this is u_N-1
        xdot = QuadrotorStateFcnBase(xk, uk);
        xN = xk + Ts * xdot;
        f  = f + (xN - x_target)' * P * (xN - x_target) + (uk-u_eq)' * R * (uk-u_eq);

        for i = 1:1:NumberofEllipses
            ella = ellipses{i}.a;
            ellb = ellipses{i}.b;
            ellh = ellipses{i}.h;  
            ellxc = ellipses{i}.xc; 
            ellyc = ellipses{i}.yc; 
            % out the ellipse
            % add terminal soft constraint
            softc1 = - ((xN(1) - ellxc)^2)/(ella + 0.01)^2 - ((xN(2) - ellyc)^2)/(ellb + 0.01)^2 +1;
            softc2 = - (0.00 + ellh) + xN(3);
            f = f + mui * (min([softc1 softc2]));
        end
    end

    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = zeros([1, 4 * N]);
    ub = ones([1, 4 * N]);

    % persistent U_initial;  
    % 
    % if isempty(U_initial)  % first initialize
    %     U_initial = kron(u_eq', ones([1, N]));  
    % end

    U_initial = kron(u_eq', ones([1, N]));

    options = optimoptions(@fmincon,'MaxFunctionEvaluations',3.0e+04);


    UU = fmincon(@mycostfun, U_initial, A,b,Aeq,beq,lb,ub,@mynlconstraints,options);

    u = UU(1:4)';

    % U_initial = UU; % update 


end % End of myMPController


