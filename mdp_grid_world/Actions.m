%% Summary
% This function takes the current state and returns the viable actions.
% Note that it takes the size of the grid to check for viability.

%% Inputs
% s: the state is a 1x2 tuple of reals
% n: the grid dimension as a real number

%% Outputs
% a: the actions is a mx2 matrix of reals with each row being a viable 
% action.  Here, m is the number of viable actions.

function a = Actions(s,n)
    % field
    if (s(1) > 1 && s(1) <n && s(2) > 1 && s(2) < n)
        a = [1,0;0,1;-1,0;0,-1];
    % lower left
    elseif (s(1) == 1 && s(2) == 1)
        a = [1,0;0,1];
    % upper left
    elseif (s(1) == 1 && s(2) == n)
        a = [-1,0;0,-1];
    % lower right
    elseif (s(1) == n && s(2) == 1)
        a = [1,0;0,-1];
    % upper right
    elseif (s(1) == n && s(2) == n)
        a = [-1,0;0,-1];
    % left edge
    elseif (s(1) == 1 && s(2) > 1 && s(2) < n)
        a = [1,0;0,1;0,-1];
    % right edge
    elseif (s(1) == n && s(2) > q && s(2) < n)
        a = [0,1;-1,0;0,-1];
    % lower edge
    elseif (s(2) == 1 && s(1) > 1 && s(1) < n)
        a = [1,0;0,1;-1,0];
    % upper edge
    elseif (s(2) == n && s(1) > 1 && s(1) < n)
        a = [1,0;-1,0;0,-1];
    % else
    else
        fprintf('Unrecognized Action {%i,%i,%i}',s(1),s(2),n);
    end
end