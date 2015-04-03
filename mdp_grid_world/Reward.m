%% Summary
% This function takes the current state and selected action and returns the
% reward for that state, action pair.

%% Inputs
% s: state is a 1x2 tuple of reals.
% a: actions is a 1x2 tuple of reals.

%% Outputs
% r: reward is a real number.

function r = Reward(s,a)
    % Rewards
    % 1: free
    % 5: ice
    % 50: hole

    R = [
        1,1,1,1;
        1,5,1,5;
        1,50,5,5;
        1,1,1,1];
    
    r = R(s(1),s(2));
end