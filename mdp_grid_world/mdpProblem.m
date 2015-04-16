classdef mdpProblem
  properties
    gamma = 0.9;  % dicount factor
    epsilon = .001; % convergence treshold
    n;  % edge size
    N;  % state size
    T;  % transition probs (N x m x N)
    R;  % reward values (N)
    A;  % action definitions
    m;  % number of actions
  end
  
  methods
    function obj = mdpProblem(N,T,R,A)
      obj.n = sqrt(N);
      obj.N = N;
      obj.T = T;
      obj.R = R;
      obj.A = A;
      obj.m = size(A,2);
    end
    
    function sol = solve(obj)
      % initialize value function
      Vsol = rand(obj.N,1);
      V_diff = Inf*ones(obj.N,1);  % nxn array.  difference between values from each iteration.
      Asol = zeros(obj.N,1);  % stored action indices

      %% Analysis
      % loop until V stops changing
      while max(max(V_diff))>obj.epsilon
        V_new = zeros(size(Vsol));  % new v for this iteration
  
        % loop over states
        for k=1:obj.N  
          % loop over actions
          new_val = zeros(1,size(obj.m,1));
          for action=1:obj.m
            new_val(action) = obj.gamma*dot(squeeze(obj.T(k,action,:)),Vsol);
          end
          
          % get max value and its index
          [max_val,i_max] = max(new_val);
          % update value and record action taken
          Asol(k) = i_max;
          V_new(k) = obj.R(k) + max_val;
        end
      
        V_diff = abs(V_new-Vsol);
        Vsol = V_new;
      end
      
      % store results
      sol.V = Vsol;
      sol.A = Asol;
      
      % map actions
      sol.Amap = zeros(obj.n,obj.n,2);
      for k=1:obj.N
        [i,j] = ind2sub([obj.n,obj.n],k);
        sol.Amap(i,j,1:2) = obj.A(Asol(k),:);
      end
    end
    
    function plot_reward(obj)
      figure
      hold on
      colormap('jet')
      imagesc(reshape(obj.R,obj.n,obj.n))
      hold off  
    end
    
    function plot_sol(obj, sol)
      figure
      hold on
      colormap('jet')
      imagesc(reshape(sol.V,obj.n,obj.n))
      [X,Y] = meshgrid(1:obj.n,1:obj.n);  % possible coordinates
      quiver(X - 0.25*sol.Amap(:,:,2),Y - 0.25*sol.Amap(:,:,1),sol.Amap(:,:,2),sol.Amap(:,:,1),0.5,'r','LineWidth',2)
      hold off      
    end
  end
  
end