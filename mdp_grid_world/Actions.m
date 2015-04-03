function actions = Actions(state,n)
    % field
    if (state(1) > 1 && state(1) <n && state(2) > 1 && state(2) < n)
        actions = [1,0;0,1;-1,0;0,-1];
    % lower left
    elseif (state(1) == 1 && state(2) == 1)
        actions = [1,0;0,1];
    % upper left
    elseif (state(1) == 1 && state(2) == n)
        actions = [-1,0;0,-1];
    % lower right
    elseif (state(1) == n && state(2) == 1)
        actions = [1,0;0,-1];
    % upper right
    elseif (state(1) == n && state(2) == n)
        actions = [-1,0;0,-1];
    % left edge
    elseif (state(1) == 1 && state(2) > 1 && state(2) < n)
        actions = [1,0;0,1;0,-1];
    % right edge
    elseif (state(1) == n && state(2) > q && state(2) < n)
        actions = [0,1;-1,0;0,-1];
    % lower edge
    elseif (state(2) == 1 && state(1) > 1 && state(1) < n)
        actions = [1,0;0,1;-1,0];
    % upper edge
    elseif (state(2) == n && state(1) > 1 && state(1) < n)
        actions = [1,0;-1,0;0,-1];
    % else
    else
        fprintf('Unrecognized Action {%i,%i,%i}',state(1),state(2),n);
    end
end