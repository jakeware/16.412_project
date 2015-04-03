clear all
close all
clc

%% Notes
% This is a test of a MDP solver on a grid world

%% Setup
n = 4;  % size of the grid
gamma = 0.9;  % dicount factor
epsilon = .01; % convergence treshold
%goal_state = [4 4]; get it from the reward?

V = rand(n);
V_diff = Inf*ones(size(V));
[X,Y] = meshgrid(1:n,1:n);

%% Analysis
while max(max(V_diff))>epsilon
  V_new = zeros(size(V));
  for k=1:numel(X)
    i = X(k);
    j = Y(k);
    if (i==goal_state(1)&&j==goal_state(2))
      V_new(i,j) = R(i,j);
      continue;
    end
    actions = % get a list of actions it can do in state (i,j)
    new_val = zeros(size(actions));
    for ai=1:numel(actions)
      act = actions(ai);
      % get the probably of each next state possible with act
      % compute gamma*dot(prob_dist,V(states of the prob dist))
      % new_val(ai) = the new value
    end
    V_new(i,j) = max(new_val) + R(i,j);
  end
  V_diff = abs(V_new-V);
  V = V_new;
  imagesc(V);
end