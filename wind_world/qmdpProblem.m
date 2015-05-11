classdef qmdpProblem
  properties
    gamma = 0.9;  % dicount factor
    m;  % number of actions
    H;  % horizon
    N;  % state size
    T;  % transition probs (N x m x N)
    Z;  % observation probs (N x m x N)
    R;  % reward values (N)
    V;  % mdp value function
  end
  
  methods
    function obj = qmdpProblem(H,T,Z,R,V)
      obj.N = size(T,1);
      obj.H = H;
      obj.T = T;
      obj.Z = Z;
      obj.R = R;
      obj.m = size(T,2);
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
    
    function path = simulate(obj,sol,b0,t)
      path.b = zeros(obj.N,t+1);
      path.b(:,1) = b0;
      path.a = zeros(1,t);
      
      for k=1:t
        % get action
        [~,vmaxi] = max(sol.Q'*path.b(:,k));
        amax = sol.A(1,vmaxi);
        path.a(k) = amax;
        
        % propogate belief
        path.b(:,k+1) = squeeze(obj.T(:,amax,:))'*path.b(:,k);
        
        % probability of o inter s
        W = obj.Z'*path.b(:,k+1);
                
        % get observation
        i_z = randsample(1:length(W),1,true,W);
        
        path.b(:,k+1) = obj.Z(:,i_z).*path.b(:,k+1);
        path.b(:,k+1) = path.b(:,k+1)/norm(path.b(:,k+1));
      end
    end

    function plot_sol(obj,path,ns)
%       figure
%       hold on
%       colormap('jet')
%       
%       % actions
%       [X,Y] = meshgrid(1:obj.n,1:obj.n);  % possible coordinates
%       quiver(X - 0.25*path.Amap(:,:,2),Y - 0.25*path.Amap(:,:,1),path.Amap(:,:,2),path.Amap(:,:,1),0.5,'r','LineWidth',2)
%       
%       % path
%       scatter(path.s(:,2),path.s(:,1),100,'k','filled')
%       
%       title('QMDP Solution')
%       xlabel('X')
%       ylabel('Y')
%       hold off

      obstacle = [
          0,0,0,0,0,0;
          0,0,0,0,0,0;
          1,1,1,1,0,0;
          1,1,1,1,0,0;
          1,1,1,1,0,0;
          1,1,1,1,0,0];
      
      [X,Y] = meshgrid(1:obj.n,1:obj.n);  % possible coordinates

      % plot belief state over time
      figure
      m = ceil(sqrt(size(path.b,2)));
      for i=1:ns
        subplot(m,m,i)
        imagesc(reshape(path.b(:,i),obj.n,obj.n));
      end
    end
  end
  
end