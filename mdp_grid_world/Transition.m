%% Summary
% This function takes the current state and selected action and returns the
% probability of being in a set of states.

%% Inputs
% s: state is a 1x2 tuple of reals.
% a: actions is a mx2 matrix of reals.
% n: the grid dimension as a real number.

%% Outputs
% t: mx3 tuple of reals, where the first 2 columns are the state and the
% 3rd column is the probability of being in that state.

function t = Transition(s,a,n)
    % Transition Probabilities
    % 80% intended action
    % 10% stay still
    % 10% overshoot
    
    % check overshoot
    %s_p = state + [zeros(n,1);a;2*a];
    %if (s_p(1,3) < 1 || s_p(1,3) > n || s_p(2,3) < 1 || s_p(2,3) > n)
        
    s_p = s + a;
    t = [s_p,1];
end