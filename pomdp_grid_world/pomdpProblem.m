classdef pomdpProblem
  properties
    gamma = .9;  % dicount factor
    m;
    N;  % state size
    H;  % horizon
    T;  % transition probs (N x m x N)
    Z;  % observation probs (N x m x N)
    R;  % reward values (N)
    
  end
  
  methods
    function obj = pomdpProblem(N,H,T,Z,R,m)
      obj.N = N;
      obj.H = H;
      obj.T = T;
      obj.Z = Z;
      obj.R = R;
      obj.m = m;
    end
    
    function sol = solve(obj)
      %  initialize value function
      V = zeros(obj.N,0);
      for action=1:obj.m
        V = [V,obj.R];
      end
      
      % loop over time
      for h=1:obj.H
        h
        Vprime = V;
        V = [];
        % loop over actions
        for action=1:obj.m
          tic
          G_a = zeros(obj.N,size(Vprime,2),obj.N+1);
          G_a_star = repmat(obj.R,1,size(Vprime,2));
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
                new_alpha(k) = obj.gamma*sum(squeeze(obj.T(k,action,:)).*obj.Z(k,:)'.*alpha);
              end
              G_a_o = [G_a_o,new_alpha];
            end
            G_a(:,:,1+obs) = G_a_o;  
          end
          toc
          
          display('end')
          tic
          G_a = sum(G_a,3); % alpha vectors to be added to our tree
          V = [V,G_a]; % union of the alpha vectors set
          toc
        end
        % optional pruning of the alpha vectors
      end

      sol.V = V;
      % TODO: sol.A = A;
    end
    
  end
  
end