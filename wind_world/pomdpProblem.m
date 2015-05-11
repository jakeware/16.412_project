classdef pomdpProblem
  properties
    gamma = .9;  % dicount factor
    N;  % number of states
    M;  % number of actions
    H;  % horizon
    T;  % transition probs (N x m x N)
    Z;  % observation probs (N x num_obs)
    R;  % reward values (m x N)
    B;  % belief set if using PBVI
    sol;  % solver type (1=full,2=larks,3=pbvi)
    ns;  % number of belief samples for PBVI
  end
  
  methods
    function obj = pomdpProblem(H,T,Z,R,varargin)
      % if b0 is provided, will use point-based value iteration
      obj.N = size(T,1);
      obj.M = size(T,2);
      obj.H = H;
      obj.T = T;
      obj.Z = Z;
      obj.R = R;
      obj.B = [];
      obj.sol = 1;
      obj.ns = 0;
      
      vin=varargin;
      for i=1:length(vin)
        if isequal(vin{i},'solver')
          sol_str = vin{i+1};
          
          % check for solver type
          if strcmp(sol_str,'full')
            display('Solver is Using Full Exact Solution')
            obj.sol = 1;
          elseif strcmp(sol_str,'larks')
            display('Solver is Using Larks Algorithm')
            obj.sol = 2;
          elseif strcmp(sol_str,'pbvi')
            display('Solver is using PBVI')
            obj.sol = 3;
            
            b0 = vin{i+2};  % get belief
            obj.ns = vin{i+3};  % get belief sample number
            fprintf('Belief Samples: %d\n',obj.ns)
            obj.B = beliefPointSetExpension(b0,T,obj.ns);
            
            %obj.B = eye(obj.N);
          end
        end
      end
    end
    
    function sol = solve(obj)
      %  initialize value function
      V = [];
      A = [];
      for action=1:obj.M
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
        
        if obj.sol == 3
          G_pbvi = zeros(obj.N,obj.M,size(obj.B,2));
        end
        
        % loop over actions
        for action=1:obj.M
          G_a = zeros(obj.N,size(Vprime,2),size(obj.Z,2)+1);
          
          G_a_star = repmat(sum(squeeze(obj.R(:,action,:)),2),1,size(Vprime,2));
          G_a(:,:,1) = G_a_star;
          
          % loop over observations
          for obs=1:size(obj.Z,2)
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
                    
          if obj.sol == 3
            % using PBVI
            for bi=1:size(obj.B,2)
              alphasumoverobs = size(obj.N,1);
              for oi=1:size(obj.Z,2)
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
        
        if obj.sol == 3
          % using PBVI
          for bi=1:size(obj.B,2)
            [~,maxact] = max(G_pbvi(:,:,bi)'*obj.B(:,bi));
            V = [V,G_pbvi(:,maxact,bi)];
            A = [A,maxact];
          end
        elseif obj.sol == 2
          % pruning of the alpha vectors (lark's filter)
          % requires MOSEK
          [V,A] = larkfilt(V,A);
        elseif obj.sol == 1
          % full solution
        else
          % nothing specified, no pruning
        end
      end

      sol.V = V;
      sol.A = A;
    end
    
    function path = simulate(obj,sol,b0,t)
      path.b = zeros(obj.N,t+1);
      path.b(:,1) = b0;
      path.a = zeros(1,t);
      
      for k=1:t
        % get action
        [~,vmaxi] = max(sol.V'*path.b(:,k));
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
    
    function plot_sol(obj,path,t,node_list,obs)            
      for i=1:t
        % plot belief state over time
        figure
        colormap('jet')
        hold on

        % obstacle region
        [X,Y] = meshgrid(1:size(obs,1),1:size(obs,2));
        scatter(reshape(X(obs==1),1,[]),reshape(Y(obs==1),1,[]),200,'ks','filled');
        
        % plot nodes
        scatter(node_list(:,1),node_list(:,2),300,path.b(1:end-1,i),'filled')

        % plot node labels
        labels = cellstr(num2str([1:size(node_list,1)]'));
        text(node_list(:,1),node_list(:,2),labels,'horizontalAlignment','center','color','w')
        
        % failure state
        %scatter(8,42,800,path.b(end,i),'s','filled');
        rectangle('Position',[1,40.25,14,3],'EdgeColor','k','LineWidth',2);
        text(8,42,strcat({'Failure: '},sprintf('%2.2f',100*path.b(end,i)),'%'),'horizontalAlignment','center');
        hold off
        
        F(i) = getframe;
      end
      close all
      
      movie(F,3,1)
      movie2avi(F,'pomdp_test1.avi','fps',1)
    end
  end
end