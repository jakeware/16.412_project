clear all
close all
clc

%% Notes
% This is a test of a MDP solver on a grid world

%% Setup
n = 10;  % size of the grid
gamma = 0.9;  % dicount factor
epsilon = .01; % convergence treshold
goal_state = [1 10];

V = rand(n);
V_diff = Inf*ones(size(V));  % nxn array.  difference between values from each iteration.
[X,Y] = meshgrid(1:n,n:-1:1)  % possible coordinates

%% Analysis
%
while max(max(V_diff))>epsilon
  V_new = zeros(size(V));
  for k=1:numel(X)
    i = X(k);
    j = Y(k);
    Vi = sub2ind(size(V),i,j);
    if (i==goal_state(1)&&j==goal_state(2))
      V_new(Vi) = Reward([i,j]);
      continue;
    end
    actions = Actions([i,j],n);  % get a list of actions it can do in state (i,j)
    new_val = zeros(1,size(actions,1));
    for ai=1:size(actions,1)
      act = actions(ai,:);
      trans = Transition([i,j],act,n);
      new_val(ai) = gamma*trans(:,3)'*V(sub2ind(size(V),trans(:,1),trans(:,2)));
    end
    V_new(Vi) = max(new_val) + Reward([i,j]);
  end
  V_diff = abs(V_new-V);
  V = V_new;
  imagesc(V);
  
  %waitforbuttonpress
end