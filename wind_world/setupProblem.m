%% Summary
% This function computes the transition, reward,
% and observation functions for some stored data.

%% Inputs
% TODO

%% Outputs
% T: transition function (N x m x N)
% R: reward function (N x m x N)
% Z: observation function (N x num_obs)

function [T,R,Z] = setupProblem(node_list,edge_list,goal,uv_mean,uv_cov,batchdata,energy,z,map)
  m = size(edge_list,1); % num actions
  N = size(node_list,1); % num states
  
  % windfield probabilities
  wmargin = [1.046 0.9]';
  wp = [];
  for i=1:batchdata.met.ncustom
    speed = batchdata.met.custom(i).speed;
    direction = batchdata.met.custom(i).direction;
    [u,v] = pol2cart(direction,speed);
    w = [u v]';
    wp = [wp,mvncdf(w-wmargin,w+wmargin,uv_mean,uv_cov)];
  end
  
  % Transition
  traj_founds = energy>0;
  traj_probs = traj_founds'*wp';
  T = zeros(N,m,N);
  for j=1:m
    T(edge_list(j,1),j,edge_list(j,2)) = .8*traj_probs(j);
  end
  
  % add loop at goal node
  T(end,end+1,end) = 1;
  
  % add self loops
%   for i=1:N
%     for j=1:m
%       if edge_list(j,1) == i
%         T(i,j,edge_list(j,2)) = T(i,j,edge_list(j,2))-.3;
%         T(i,j,i) = 0.3;
%       end
%     end
%   end
  
  % add failure state
  ind = size(T,3);
  for i=1:size(T,1)
    for j=1:size(T,2)
      T(i,j,ind+1) = 1 - sum(T(i,j,:));
    end
  end
  T(end+1,:,:) = [zeros(size(T,2),size(T,3)-1),ones(size(T,2),1)];
  
  % Reward
  % negative reward for the expected energy
  exp_energy = energy'*wp';
  R = zeros(N,m,N);
  for i=1:N
    for j=1:m
      if edge_list(j,1) == i
        R(i,j,edge_list(j,2)) = -exp_energy(j);
      end
    end
  end
  
  % positive reward for getting to the goal
  for i=1:N
    for j=1:m
      if R(i,j,goal) ~= 0
        R(i,j,goal) = R(i,j,goal)+100*max(max(max(abs(R))));
      end
    end
  end
  
  % add reward for goal loop
  R(goal,end+1,goal) = 1000;
  
  % add reward for failure state
  R(:,:,end+1) = -1000;
  R(end+1,:,:) = -1000;
  
  % Z from Naomi
  Z = createObservationMatrix(node_list,edge_list,z,map);
  
  % add failure state
  Z(:,end+1) = zeros(size(Z,1),1);
  Z(end+1,:) = .01*ones(size(Z,2),1);
  Z(end,end) = 1;
  
  % normalize
  for i=1:size(Z,1)
    Z(i,:) = Z(i,:)/sum(Z(i,:));
  end
end