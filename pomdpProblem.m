classdef pomdpProblem
  properties
    gamma = .9;  % dicount factor
    m;
    n;  % gris size
    N;  % state size
    H;  % horizon
    T;  % transition probs (N x m x N)
    Z;  % observation probs (N x m x N)
    R;  % reward values (N)
    A;  % action set
    B;  % belief set if using PBVI
  end
  
  methods
    function obj = pomdpProblem(n,H,T,Z,R,A,b0)
      % if b0 is provided, will use point-based value iteration
      obj.n = n;
      obj.N = n^2;
      obj.H = H;
      obj.T = T;
      obj.Z = Z;
      obj.R = R;
      obj.m = size(A,1);
      obj.A = A;
      if (nargin>6)
        obj.B = beliefPointSetExpension(b0,T,n^2);
        %obj.B = eye(n^2);
      else
        obj.B = [];
      end
    end
    
    function sol = solve(obj)
      %  initialize value function
      V = [];
      A = [];
      for action=1:obj.m
        V = [V,sum(squeeze(obj.R(:,action,:)),2)];
        A = [A,action];
      end
            
      % loop over time
      for h=1:obj.H
        fprintf('Iteration: %i\n',h);
        Vprime = V;
        V = [];
        Aprime = A;
        A = [];
        
        if size(obj.B,2)>0
          G_pbvi = zeros(obj.N,obj.m,size(obj.B,2));
        end
        
        % loop over actions
        for action=1:obj.m
          G_a = zeros(obj.N,size(Vprime,2),obj.N+1);
          
          G_a_star = repmat(sum(squeeze(obj.R(:,action,:)),2),1,size(Vprime,2));
          G_a(:,:,1) = G_a_star;
          
          % loop over observations
          for obs=1:obj.N
            G_a_o = [];
            
            % loop over alpha vectors
            for alphai=1:size(Vprime,2)
              alpha = Vprime(:,alphai);
              new_alpha = zeros(obj.N,1);
              % loop over states
              for k=1:obj.N
                new_alpha(k) = obj.gamma*sum(squeeze(obj.T(k,action,:)).*obj.Z(:,obs).*alpha);
              end
              G_a_o = [G_a_o,new_alpha];
            end
          
            G_a(:,:,1+obs) = G_a_o;
          end
                    
          if size(obj.B,2)>0
            % using PBVI
            for bi=1:size(obj.B,2)
              alphasumoverobs = size(obj.N,1);
              for oi=1:obj.N
                G_a_o = squeeze(G_a(:,:,oi));
                [~,maxalphabi] = max(G_a_o'*obj.B(:,bi));
                maxalpha = G_a_o(:,maxalphabi);
                alphasumoverobs = alphasumoverobs + maxalpha;
              end
              G_a_b = sum(squeeze(obj.R(:,action,:)),2) + alphasumoverobs;
              G_pbvi(:,action,bi) = G_a_b;
            end
          else
            G_a = sum(G_a,3); % standard cross-sum 
            V = [V,G_a]; % union of the alpha vectors set
            G_a_actions = [repmat(action,1,size(Aprime,2));Aprime];
            A = [A,G_a_actions];
          end
        end
        
        if size(obj.B,2)>0
          % using PBVI
          for bi=1:size(obj.B,2)
            [~,maxact] = max(G_pbvi(:,:,bi)'*obj.B(:,bi));
            V = [V,G_pbvi(:,maxact,bi)];
            A = [A,maxact];
          end
        else
          % pruning of the alpha vectors (lark's filter)
          % requires MOSEK
          [V,A] = larkfilt(V,A);
        end
      end

      sol.V = V;
      sol.A = A;
    end
    
    function path = simulate(obj,sol,b0,t)
      path.b = zeros(obj.N,t+1);
      path.p = zeros(t+1,1);
      path.Amap = zeros(obj.n,obj.n,2);
      path.s = zeros(t+1,2);
      
      path.b(:,1) = b0;
      [p_s,i_s] = max(b0);
      [i,j] = ind2sub([obj.n,obj.n],i_s);
      path.s(1,:) = [i,j];
      path.p(1) = p_s;
      i_a = zeros(t,1); t = 12;
      
      for k=1:t
        % get action
        [vi,ai] = max(sol.V'*path.b(:,k));
        i_a(k) = sol.A(1,ai);
        path.a(k,:) = obj.A(i_a(k),:);
        
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
      
      for i=1:t
        path.Amap(path.s(i,1),path.s(i,2),1:2) = path.a(i,:);
        path.Pmap(path.s(i,1),path.s(i,2)) = path.p(i,:);
      end
    end
    
    function plot_sol(obj,path)
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
      
      figure
      % plot belief state over time
      for i=1:size(path.b,2)
        imagesc(reshape(path.b(:,i),obj.n,obj.n));
        F(i) = getframe;
      end

      movie(F,3,1)
    end
  end
  
end