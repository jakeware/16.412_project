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
  for k=1:numel(V)
    % subscript for this state
    x_k = X(k);
    y_k = Y(k);
    
    % convert subscript to index
    i_v = sub2ind(size(V),x_k,y_k);
    
    % get a list of actions it can do in state (i,j)
    actions = Actions([x_k,y_k],n);
    
    %pause;
    
    % iterate over each possible action in this state
    new_val = zeros(1,size(actions,1));
    for i_act=1:size(actions,1);
      act = actions(i_act,:);
      trans = Transition([x_k,y_k],act,n);
      
      % sum transition probabilities times current values and discount rate
      new_val(i_act) = gamma*trans(:,3)'*V(sub2ind(size(V),trans(:,1),trans(:,2)));
      
      %pause;
    end
    
    % get max value and its index
    [max_val,i_max] = max(new_val);
    
    % update value and record action taken
    XY(x_k,y_k,1:2) = [x_k,y_k];
    A(x_k,y_k,1:2) = actions(i_max,:);
    V_new(i_v) = max_val + Reward([x_k,y_k]);
  end
  
  %pause;
  
  V_diff = abs(V_new-V);
  V = V_new;
end


%% Plot
figure
hold on
colormap('gray')
imagesc(flipud(V));
quiver(XY(:,:,1) - 0.25*A(:,:,1),XY(:,:,2) - 0.25*A(:,:,2),A(:,:,1),A(:,:,2),0.5,'r','LineWidth',2)
hold off