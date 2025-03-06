function param = mySetup(shape)
%% Modify the following function for your setup function
    param.Ts = 0.14;     % This is a sample way to set your sampling time    
end % End of mySetup
function u = myMPController(param, y)
%% Design your controller here
    u = zeros(4,1);      % Create the output array of the appropriate size    
end % End of myMPController