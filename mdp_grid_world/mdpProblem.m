classdef mdpProblem
  properties
    gamma = 0.9;  % dicount factor
    epsilon = .001; % convergence treshold
    N;  % state size
    T;  % transition probs (N x m x N)
    R;  % reward values (N)
    m;  % number of actions
  end
  
  methods
    function obj = mdpProblem(N,T,R,m)
      obj.N = N;
      obj.T = T;
      obj.R = R;
      obj.m = m;
    end
    
    function sol = solve(obj)
      % initialize value function
      V = rand(obj.N,1);
      V_diff = Inf*ones(obj.N,1);  % nxn array.  difference between values from each iteration.

      %% Analysis
      % loop until V stops changing
      while max(max(V_diff))>obj.epsilon
        V_new = zeros(size(V));  % new v for this iteration
  
        % loop over actions
        for action=1:m
          % loop over states
          for k=1:obj.N  
              new_val(action) = obj.gamma*T(k,action,:)'*V;
          end
            
          % get max value and its index
          [max_val,i_max] = max(new_val);
          % update value and record action taken
          A(i,j,1:2) = actions(i_max,:);
          V_new(i,j) = max_val + obj.reward_function([i,j]);
        end
      
        V_diff = abs(V_new-V);
        V = V_new;
      end
      
      sol.Values = V;
      sol.Actions = A;
    end
    
    function plot(obj, sol)
      %% Plot
      figure
      hold on
      colormap('jet')
      imagesc(sol.Values)
      [X,Y] = meshgrid(1:obj.grid_size,1:obj.grid_size);  % possible coordinates
      quiver(X - 0.25*sol.Actions(:,:,2),Y - 0.25*sol.Actions(:,:,1),sol.Actions(:,:,2),sol.Actions(:,:,1),0.5,'r','LineWidth',2)
      hold off      
    end
  end
  
end