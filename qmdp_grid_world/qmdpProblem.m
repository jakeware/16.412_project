classdef qmdpProblem
  properties
    gamma = .9;  % dicount factor
    m;
    H;  % horizon
    N;  % state size
    T;  % transition probs (N x m x N)
    Z;  % observation probs (N x m x N)
    R;  % reward values (N)
    V;  % mdp value function
  end
  
  methods
    function obj = qmdpProblem(N,H,T,Z,R,m,V)
      obj.N = N;
      obj.H = H;
      obj.T = T;
      obj.Z = Z;
      obj.R = R;
      obj.m = m;
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
      % TODO: sol.A = A;
    end
    
  end
  
end