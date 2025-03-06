function [ course, fig ] = defaultCourse( dis, coursenum )
%DEFAULTCOURSE Generate the default course for the core coursework

if coursenum == 1
    
    constraints.rect.bot = [0.00, 0.05;
                            0.45, 0.50;
                            0.50, 0.45;
                            0.05, 0.00];
    constraints.rect.h= -0.6;
    start  = [0.05, 0.05, -0.05];
    target = [0.4, 0.4, -0.4];

    constraints.ellipses = {};
    
    
    
    
    Tt   = 5;
    Te   = 1;
    ThetaMax = 0.1;
    course.perturbSize = 0.0;


    
elseif coursenum == 2
    % The course for the second part
    constraints.rect.bot = [0.00, 0.05;
                            0.25, 0.30;
                            0.50, 0.05;
                            0.25, -0.20];
    constraints.rect.h=-0.4;
    
    ellipse.a  = 0.3;
    ellipse.b  = 0.3;
    ellipse.h  = -0.35;
    ellipse.xc = 0.25;
    ellipse.yc = -0.23;
    constraints.ellipses{1} = ellipse;

    
    start  = [0.05, 0.05, -0.05];
    target = [0.45, 0.05, -0.30];
    
    
    Tt   = 5;
    Te   = 1;
    ThetaMax = 0.2;
    course.perturbSize = 0.1;
end

shape.constraints = constraints;

shape.eps_t       = 0.02;
shape.start       = start;
shape.target      = target;
shape.Tt          = Tt;
shape.Te          = Te;
shape.ThetaMax    = ThetaMax;

% The default shape is always considered problem 1
course.prob      = 1;
course.shape     = shape;
course.completed = 0;

% Create the perturbations
rng(15012);
randPerturb = @() ( ( rand()-0.5 ) * course.perturbSize ) + 1;

course.perturb.IxxVal        = randPerturb();
course.perturb.IyyVal        = randPerturb();
course.perturb.IzzVal        = randPerturb();
course.perturb.kVal          = randPerturb();
course.perturb.lVal          = randPerturb();
course.perturb.mVal          = randPerturb();
course.perturb.bVal          = randPerturb();
course.perturb.omegaMax2Val  = randPerturb();

fig = plotCourse( course, 'Default course ', dis );
end
