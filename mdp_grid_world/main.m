clear all
close all
clc

%% Notes
% This is a test of a MDP solver on a grid world

%% Setup
n = 4;  % size of the grid
gamma = 0.9;  % dicount factor
epsilon = .01; % convergence treshold
goal_state = [4 4];

V = rand(n);
V_diff = Inf*ones(size(V));  % nxn array.  difference between values from each iteration.
[X,Y] = meshgrid(1:n,n:-1:1);  % possible coordinates
A = zeros([size(V),2]);  % array of selected actions

%% Analysis
% loop until V stops changing
while max(max(V_diff))>epsilon
  V_new = zeros(size(V));  % new v for this iteration
  
  % for each state
  for k=1:n^2
    % subscript for this state
    i_x = X(k);
    i_y = Y(k);
    
    % convert subscript to index
    i_xy = sub2ind(size(V),i_x,i_y);
    
    % get a list of actions it can do in state (i,j)
    actions = Actions([i_x,i_y],n);
    
    % iterate over each possible action in this state
    new_val = zeros(1,size(actions,1));
    for i_act=1:size(actions,1)
      act = actions(i_act,:);
      trans = Transition([i_x,i_y],act,n);
      
      % sum transition probabilities times current values
      new_val(i_act) = gamma*trans(:,3)'*V(sub2ind(size(V),trans(:,1),trans(:,2)));
    end
    
    % get max value and its index
    [max_val,i_max] = max(new_val);
    
    % update value and record action taken
    XY(i_x,i_y,1:2) = [i_x,i_y]; 
    A(i_x,i_y,1:2) = actions(i_max,:);
    V_new(i_xy) = max_val + Reward([i_x,i_y]);
  end
  
  V_diff = abs(V_new-V);
  V = V_new;
end


%% Plot
figure
hold on
imagesc(flipud(V));
quiver(XY(:,:,1),XY(:,:,2),A(:,:,1),A(:,:,2),0.5,'k')
hold off