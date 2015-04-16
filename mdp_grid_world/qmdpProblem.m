classdef qmdpProblem
  properties
    gamma = 0.9;  % dicount factor
    m;  % number of actions
    H;  % horizon
    n;  % grid size
    N;  % state size
    T;  % transition probs (N x m x N)
    Z;  % observation probs (N x m x N)
    R;  % reward values (N)
    A;  % action set
    V;  % mdp value function
  end
  
  methods
    function obj = qmdpProblem(n,H,T,Z,R,A,V)
      obj.n = n;
      obj.N = n^2;
      obj.H = H;
      obj.T = T;
      obj.Z = Z;
      obj.R = R;
      obj.m = size(A,1);
      obj.A = A;
      obj.V = V;
    end
    
    function sol = solve(obj)
      % initialize value function
      Q = zeros(obj.N,obj.m);
      
      % loop over actions
      for action=1:obj.m
        % loop over states
        for k=1:obj.N
          Q(k,action) = obj.R(k) + dot(obj.V,squeeze(obj.T(k,action,:)));
        end
      end
      
      sol.Q = Q;
    end
    
    function path = simulate(obj,sol,b0)
      b(:,1) = b0;
      [p_s,i_s] = max(b0);
      path(1,:) = ind2sub([obj.n,obj.n],i_s);
      path(1,3) = p_s;
      
      for i=1:obj.H
        % get action
        [vi,ai] = max(sol.Q'*b(:,i))
        i_a(i) = ai;
        a(i,:) = obj.A(ai,:);
        
        % propogate belief       
        b(:,i+1) = obj.Z(:,:)*squeeze(obj.T(:,i_a(i),:)/sum(sum(obj.T(:,i_a(i),:))))*b(:,i);
        
        % store MAP state
        [p_s,i_s] = max(b(:,i+1));
        path(i+1,:) = ind2sub([obj.n,obj.n],i_s);
        path(i+1,3) = p_s;
      end
    end
      
    function plot_sol(obj,sol,path)
      figure
      hold on
      colormap('jet')
      
      % value function
      %imagesc(reshape(sol.V,obj.n,obj.n))
      
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