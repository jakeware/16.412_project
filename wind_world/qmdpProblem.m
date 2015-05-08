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
    
    function path = simulate(obj,sol,b0,ns)
      path.b = zeros(obj.N,obj.H+1);
      path.p = zeros(obj.H+1,1);
      path.Amap = zeros(obj.n,obj.n,2);
      path.s = zeros(obj.H+1,2);
      
      path.b(:,1) = b0;
      [p_s,i_s] = max(b0);
      [i,j] = ind2sub([obj.n,obj.n],i_s);
      path.s(1,:) = [i,j];
      path.p(1) = p_s;
      i_a = zeros(obj.H,1);
      
      for k=1:ns
        % get action
        [vi,ai] = max(sol.Q'*path.b(:,k));
        i_a(k) = ai;
        path.a(k,:) = obj.A(ai,:);
        
        % propogate belief
        path.b(:,k+1) = squeeze(obj.T(:,i_a(k),:))'*path.b(:,k);
        
        % probability of o inter s
        W = obj.Z'*path.b(:,k+1);

        % get observation
        i_z = randsample(1:length(W),1,true,W);
        
        path.b(:,k+1) = obj.Z(:,i_z).*path.b(:,k+1);
        path.b(:,k+1) = path.b(:,k+1)/norm(path.b(:,k+1));
        
        % store MAP state
        [p_s,i_s] = max(path.b(:,k+1));
        [i,j] = ind2sub([obj.n,obj.n],i_s);
        path.s(k+1,:) = [i,j];
        path.p(k+1) = p_s;
      end
      
      for i=1:obj.H
        path.Amap(path.s(i,1),path.s(i,2),1:2) = path.a(i,:);
        path.Pmap(path.s(i,1),path.s(i,2)) = path.p(i,:);
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