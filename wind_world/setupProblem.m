%% Summary
% This function computes the transition, reward,
% and observation functions for some stored data.

%% Inputs
% none

%% Outputs
% T: transition function (N x m x N)
% R: reward function (N x m x N)
% Z: observation function (N x num_obs)

function [T,R,Z] = setupProblem(node_list,edge_list,obs,goal,uv_mean,uv_cov,setup_data,energy_data)
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
  traj_founds = energy_data.energy>0;
  traj_probs = traj_founds'*wp';
  T = zeros(N,m,N);
  for j=1:m
    T(edge_list(j,1),j,edge_list(j,2)) = traj_probs(j);
  end
  
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
  createObservationMatrix(node_list,edge_list,obs);

  % return values
  %sol.A = A;
  %sol.T = T;
  %sol.R = R;
  %sol.Z = Z;
end