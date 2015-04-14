%% Summary
% This function takes the current state and returns a probability
% distribution over possible states.

%% Inputs
% s: the state is a 1x2 tuple of reals
% n: the grid dimension as a real number
% C: an nxn matrix of points with no state estimate

%% Outputs
% e: the observation is a mx3 matrix of reals with the first two columns of
% each row being a state and the third column its associated probability.  ]
% Here, m is the number of viable actions.

function e = Observation(s,n,C)
    e = zeros(numel(C),3);

    % are we in a blind spot?
    if C(s(1),s(2)) == 1
        for i=1:n
            for j=1:n
                e(n*(i-1) + j,1:3) = [i,j,1/numel(C)];
            end
        end
        
        return
    % perfect estimate
    else
        e = [s(1),s(2),1];
        return
    end
end