%% Summary
% This function takes the current state and selected action and returns the
% reward for that state, action pair.

%% Inputs
% s: state is a 1x2 tuple of reals.
% a: actions is a 1x2 tuple of reals.

%% Outputs
% r: reward is a real number.

function r = Reward(n,s)
    % hardcoded
%     R = [
%         1,1,1,1;
%         1,5,10,20;
%         1,5,100,20;
%         1,5,10,1];

    R = ones(n);
    R(:,1) = 5*R(:,1);
    %R(2,:) = 10*R(:,n);
    R(end,end) = 50;
    
    r = R(s(1),s(2));
end