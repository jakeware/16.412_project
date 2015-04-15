clear all
close all
clc

%% Setup
% A up, right, down, left
n = 2;
m = 4;
H = 5;
T(1,:,:) = [1,0,0,0;
            0,1,0,0;
            0,0,1,0;
            1,0,0,0];
          
T(2,:,:) = [1,0,0,0;
            0,0,0,1;
            0,1,0,0;
            0,1,0,0];
          
T(3,:,:) = [0,0,1,0;
            0,0,1,0;
            0,0,0,1;
            1,0,0,0];
          
T(4,:,:) = [0,0,1,0;
            0,0,0,1;
            0,0,0,1;
            0,1,0,0]; 
  
Z = eye(n^2);
R = ones(n);
R(2,2) = 5*R(2,2);
R = reshape(R,n^2,1);

%% Execution
% solve mdp
mdp_prob = mdpProblem(n,@(s)Reward(n,s));
mdp_sol = mdp_prob.solve();

% solve qmdp
qmdp_prob = qmdpProblem(n^2,H,T,Z,R,m,reshape(mdp_sol.Values,n^2,1));
qmdp_sol = qmdp_prob.solve();