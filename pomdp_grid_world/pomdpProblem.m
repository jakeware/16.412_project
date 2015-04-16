classdef pomdpProblem
  properties
    gamma = 1;  % dicount factor
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
      V = zeros(obj.N,obj.m);
      A = zeros(0,obj.m);
      %for action=1:obj.m
      %  V = [V,obj.R];
      %  A = [A,action];
      %end
      
%       sol.V = V;
%       sol.A = A;
%       return;
      
      % loop over time
      for h=1:obj.H
        fprintf('Iteration: %i\n',h);
        Vprime = V;
        V = [];
        Aprime = A;
        A = [];
        % loop over actions
        for action=1:obj.m
%           action
                        
          G_a = zeros(obj.N,size(Vprime,2),obj.N+1);
          G_a_star = repmat(obj.R,1,size(Vprime,2));
          G_a(:,:,1) = G_a_star;
          % loop over observations
          for obs=1:obj.N
            G_a_o = [];
            % loop over alpha vectors
            for alphai=1:size(Vprime,2)
%               alphai
              
              alpha = Vprime(:,alphai);
              new_alpha = zeros(obj.N,1);
              % loop over states
              for k=1:obj.N
%                 k
%                 obj.gamma
%                 squeeze(obj.T(k,action,:))
%                 obj.Z(k,:)
%                 alpha
%                 squeeze(obj.T(k,action,:)).*obj.Z(k,:)'.*alpha
                
                new_alpha(k) = obj.gamma*sum(squeeze(obj.T(k,action,:)).*obj.Z(:,obs).*alpha);
                
%                 pause
              end
%               new_alpha

              G_a_o = [G_a_o,new_alpha];
%               pause
            end
            
            G_a(:,:,1+obs) = G_a_o;  
          end
          
          G_a = sum(G_a,3); % alpha vectors to be added to our tree
          V = [V,G_a]; % union of the alpha vectors set
          
          G_a_actions = [Aprime;repmat(action,1,size(Aprime,2))];
          A = [A,G_a_actions];
        end
        % optional pruning of the alpha vectors
        
      end

      sol.V = V;
      sol.A = A;
    end
    
  end
  
end