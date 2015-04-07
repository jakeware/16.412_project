classdef mdpProblem
  properties
    gamma = 0.9;  % dicount factor
    epsilon = .01; % convergence treshold

    grid_size;
    reward_function;
  end
  
  methods
    function obj = mdpProblem(grid_size, reward_function)
      obj.grid_size = grid_size;
      obj.reward_function = reward_function;
    end
    
    function sol = solve(obj)
      V = rand(obj.grid_size);
      V_diff = Inf*ones(size(V));  % nxn array.  difference between values from each iteration.
      A = zeros([size(V),2]);  % array of selected actions

      %% Analysis
      % loop until V stops changing
      while max(max(V_diff))>obj.epsilon
        V_new = zeros(size(V));  % new v for this iteration
  
        % for each state
        for i=1:obj.grid_size
          for j=1:obj.grid_size
            % get a list of actions it can do in state (i,j)
            actions = Actions([i,j],obj.grid_size);
            
            % iterate over each possible action in this state
            new_val = zeros(1,size(actions,1));
            for i_act=1:size(actions,1);
              act = actions(i_act,:);
              trans = Transition([i,j],act,obj.grid_size);      
              % sum transition probabilities times current values and discount rate
              new_val(i_act) = obj.gamma*trans(:,3)'*V(sub2ind(size(V),trans(:,1),trans(:,2)));
            end
            % get max value and its index
            [max_val,i_max] = max(new_val);
            % update value and record action taken
            A(i,j,1:2) = actions(i_max,:);
            V_new(i,j) = max_val + Reward([i,j]);
          end
        end
      
        V_diff = abs(V_new-V);
        V = V_new;
      end
      
      sol.Values = V;
      sol.Actions = A;
    end
    
    function plot(obj, sol)
      %% Plot
      figure
      hold on
      colormap('gray')
      imagesc(sol.Values);
      [X,Y] = meshgrid(1:obj.grid_size,1:obj.grid_size);  % possible coordinates
      quiver(X - 0.25*sol.Actions(:,:,1),Y - 0.25*sol.Actions(:,:,2),sol.Actions(:,:,1),sol.Actions(:,:,2),0.5,'r','LineWidth',2)
      hold off      
    end
  end
  
end