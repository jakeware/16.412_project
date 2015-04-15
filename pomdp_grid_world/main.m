clear all
close all
clc

%% Setup
% Ns = 2;  % number of states
% Na = 2;  % number of actions
% 
% State Space
% State = reshape(1:Ns^2,Ns,Ns);
% 
% Actions
% Actions = [
%   1,0;  % increment row
%   0,-1;  % increment column
%   -1,0;  % decrement row 
%   0,1;  % decrement column
%   0,0];  % stay
% 
% Rewards
% R = ones(Ns);
% R(:,1) = 5*R(:,1);
% R(2,:) = 10*R(:,Ns);
% R(end,end) = 50;
% Rewards = reshape(R,1,Ns^2);
% 
% Observations
% Observations = zeros(Ns^2,Ns^2);
% 
% Transitions
% Transitions = zeros(Ns^2,Na,Ns^2);
% 
% fill in arrays
% for i=1:Ns
%   for j=1:Ns
%     field
%     if (i > 1 && i < Ns && j > 1 && j < Ns)
%       transitions  
%       a = [0,0;1,0;0,1;-1,0;0,-1];
%       subs = [i,j] + a;
%       
%       observations
%         
%     upper left
%     elseif (i == 1 && j == 1)
%         a = [0,0;1,0;0,1];
%     upper right
%     elseif (i == 1 && j == Ns)
%         a = [0,0;1,0;0,-1];
%     lower left
%     elseif (i == Ns && j == 1)
%         a = [0,0;-1,0;0,1];
%     lower right
%     elseif (i == Ns && j == Ns)
%         a = [0,0;-1,0;0,-1];
%     upper edge
%     elseif (i == 1 && j > 1 && j < Ns)
%         a = [0,0;1,0;0,1;0,-1];
%     lower edge
%     elseif (i == Ns && j > 1 && j < Ns)
%         a = [0,0;0,1;-1,0;0,-1];
%     left edge
%     elseif (j == 1 && i > 1 && i < Ns)
%         a = [0,0;1,0;0,1;-1,0];
%     right edge
%     elseif (j == Ns && i > 1 && i < Ns)
%         a = [0,0;1,0;-1,0;0,-1];
%     end
%   end
% end

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
R(2,2) = 5*R(1,1);
R = reshape(R,n^2,1);

%% Execution
prob = pomdpProblem(n^2,H,T,Z,R,m);
sol = prob.solve();