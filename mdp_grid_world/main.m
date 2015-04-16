clear all
close all
clc

%% Setup
% grid size
n = 2;

% Actions
A = [
  -1,0;  % decrement row (up)
  0,1;  % increment column (right)
  1,0;  % increment row (down)
  0,-1;  % decrement column (left)
  0,0];  % stay

% Transitions
T(1,:,:) = [1,0,0,0;
            0,0,1,0;
            0,1,0,0;
            1,0,0,0;
            1,0,0,0];
          
T(2,:,:) = [1,0,0,0;
            0,0,0,1;
            0,1,0,0;
            0,1,0,0;
            0,1,0,0];
          
T(3,:,:) = [0,0,1,0;
            0,0,1,0;
            0,0,0,1;
            1,0,0,0;
            0,0,1,0];
          
T(4,:,:) = [0,0,1,0;
            0,0,0,1;
            0,0,0,1;
            0,1,0,0;
            0,0,0,1]; 

% Rewards
R = ones(n);
R(2,2) = 5*R(2,2);
R = reshape(R,n^2,1);

%% Exectuion
prob = mdpProblem(n^2,T,R,A);
prob.plot_reward();
sol = prob.solve();
prob.plot_sol(sol);