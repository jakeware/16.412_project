%% Summary
% This function computes the transition, reward,
% and observation functions for some stored data.

%% Inputs
% none

%% Outputs
% T: transition function (N x m x N)
% R: reward function (N x m x N)
% Z: observation function (N x num_obs)

function R = setupProblem()

  % those should be arguments
  % node locations (x, y)
  node_list = [
    30,20;
    100,100;
    195,40;
    195,180;
    10,180];
  % graph edges (node index, node index)
  edge_list = [
    1,2;
    1,3;
    1,5;
    2,3;
    2,4;
    3,2;
    3,4;
    5,2;
    5,4;
    ];
  goal = 5;
  n = 100;
  uv_mean = [-4, 4]';  % x-vel [m/s]
  uv_cov = [10, -3; 
            -3,  5];
  setup_data = load('batchdata');
  energy_data = load('energy.mat');

  m = size(edge_list,1); % num actions
  N = size(node_list,1); % num states
  
  % windfield probabilities
  wmargin = [.05 .05]';
  wp = [];
  for i=1:setup_data.batchdata.met.ncustom
    speed = setup_data.batchdata.met.custom(i).speed;
    direction = setup_data.batchdata.met.custom(i).direction;
    [u,v] = pol2cart(direction,speed);
    w = [u v]';
    wp = [wp,mvncdf(w-wmargin,w+wmargin,uv_mean,uv_cov)];
  end
  
  % Transition
  
  % Reward
  % negative reward for the expected energy
  R2d = repmat(-energy_data.energy'*wp',1,N);
  % positive reward for getting to the goal
  R2d(:,goal) = R2d(:,goal)+10*max(max(abs(R2d)));
  R = zeros(N,m,N);
  for i=1:N
    R(i,:,:) = R2d;
  end
  
  % Z from Naomi

  % return values
  %sol.A = A;
  %sol.T = T;
  %sol.R = R;
  %sol.Z = Z;
end