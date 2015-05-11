classdef mdpProblem
  properties
    gamma = 0.9;  % dicount factor
    epsilon = .001; % convergence treshold
    n;  % edge size
    H;  % simulation horizon
    N;  % state size
    T;  % transition probs (N x m x N)
    R;  % reward values (N)
    m;  % number of actions
  end
  
  methods
    function obj = mdpProblem(T,R)
      obj.N = size(T,1);
      obj.T = T;
      obj.R = R;
      obj.m = size(T,2);
    end
    
    function sol = solve(obj)
      % initialize value function
      Vsol = rand(obj.N,1);
      V_diff = Inf*ones(obj.N,1);  % nxn array.  difference between values from each iteration.
      Asol = zeros(obj.N,1);  % stored action indices

      % loop until V stops changing
      while max(max(V_diff))>obj.epsilon
        V_new = zeros(size(Vsol));  % new v for this iteration
  
        % loop over states
        for k=1:obj.N  
          % loop over actions
          new_val = zeros(1,size(obj.m,1));
          for action=1:obj.m
            new_val(action) = dot(squeeze(obj.T(k,action,:)),squeeze(obj.R(k,action,:))) + obj.gamma*dot(squeeze(obj.T(k,action,:)),Vsol);
          end
          
          % get max value and its index
          [max_val,i_max] = max(new_val);
          % update value and record action taken
          Asol(k) = i_max;
          V_new(k) = max_val;
        end
      
        V_diff = abs(V_new-Vsol);
        Vsol = V_new;
      end
      
      % store results
      sol.V = Vsol;
    end
    
    function path = simulate(obj,sol,s0,H)
      i_s0 = sub2ind([obj.n,obj.n],s0(1),s0(2));
      path(1,:) = s0;
      
      for i=2:H
        % get action
        i_a = path(i-1,:);
        a(i,1) = sol.Amap(i_a(1),i_a(2),1);
        a(i,2) = sol.Amap(i_a(1),i_a(2),2);
        
        % store new state
        path(i,:) = path(i-1,:) + a(i,:);
      end
    end
      
    function plot_sol(obj, sol, path)
      figure
      hold on
      colormap('jet')
      
      % value function
      imagesc(reshape(sol.V,obj.n,obj.n))
      
      % actions
      [X,Y] = meshgrid(1:obj.n,1:obj.n);  % possible coordinates
      quiver(X - 0.25*sol.Amap(:,:,2),Y - 0.25*sol.Amap(:,:,1),sol.Amap(:,:,2),sol.Amap(:,:,1),0.5,'r','LineWidth',2)
      
      % path
      scatter(path(:,2),path(:,1),100,'k','filled')
      
      title('MDP Solution')
      xlabel('X')
      ylabel('Y')
      hold off      
    end
    
  end
  
end