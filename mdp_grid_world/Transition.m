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
    
    % new state (stay,intended action,overshoot)
    s_p = repmat(s,3,1) + [zeros(1,2);a;2*a];
    
    % check overshoot
    if (s_p(3,1) < 1 || s_p(3,1) > n || s_p(3,2) < 1 || s_p(3,2) > n)
        % check action
        if (s_p(2,1) < 1 || s_p(2,1) > n || s_p(2,2) < 1 || s_p(2,2) > n)
            % return only the stay action
            t = [s_p(1,:),1];
        else
            % return stay,intended
            t = [s_p(1:2,:),[0.1;0.9]];
        end
    else
        % return stay,intended,overshoot
        t = [s_p,[0.1;0.8;0.1]];
    end
    
    % ignore probabilities
    %s_p = s + a;
    %t = [s_p,1];
end