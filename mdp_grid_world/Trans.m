function trans = Trans(s,a,n)
    % Transition Probabilities
    % 80% intended action
    % 10% stay still
    % 10% overshoot
    
    % check overshoot
    s_p = state + [zeros(n,1),a,2*a]
    %if (s_p(1) < 1 || s_p(1) > n || s_p(2) < 1 || s_p(2) > n)
        
end