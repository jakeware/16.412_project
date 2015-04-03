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
    x = s(1);
    y = s(2);

    % field
    if (x > 1 && x <n && y > 1 && y < n)
        a = [1,0;0,1;-1,0;0,-1];
    % lower left
    elseif (x == 1 && y == 1)
        a = [1,0;0,1];
    % upper left
    elseif (x == 1 && y == n)
        a = [1,0;0,-1];
    % lower right
    elseif (x == n && y == 1)
        a = [-1,0;0,1];
    % upper right
    elseif (x == n && y == n)
        a = [-1,0;0,-1];
    % left edge
    elseif (x == 1 && y > 1 && y < n)
        a = [1,0;0,1;0,-1];
    % right edge
    elseif (x == n && y > 1 && y < n)
        a = [0,1;-1,0;0,-1];
    % lower edge
    elseif (y == 1 && x > 1 && x < n)
        a = [1,0;0,1;-1,0];
    % upper edge
    elseif (y == n && x > 1 && x < n)
        a = [1,0;-1,0;0,-1];
    % else
    else
        fprintf('Unrecognized Action {%i,%i,%i}',x,y,n);
    end
end