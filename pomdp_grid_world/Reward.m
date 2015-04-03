function reward = Reward(state,actions)
    % Rewards
    % 1: free
    % 5: ice
    % 50: hole

    R = [
        1,1,1,1;
        1,5,1,5;
        1,50,5,5;
        1,1,1,1];
    
    reward = R(state(1),state(2));
end