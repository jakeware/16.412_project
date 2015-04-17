clear all
close all
clc

%% Setup
run_complexity = 1;
plot_complexity = 1;

%% Space and Time Complexity
if run_complexity
  n_max = 3;
  H = 6;  % solver horizon
  sim_time = 12;  % simulation steps

  % result vectors
  state_size = (2:n_max).^2;
  run_time = zeros(5,1:n_max-1);

  % iteratore over grid size
  for n=2:3
    n
    
    % get actions and compute reward, transition, and observation functions
    inp = setupComplexity(n);

    % mdp
    mdp_prob = mdpProblem(n,inp.T,inp.R,inp.A);
    tic
    mdp_prob.solve();
    run_time(1,n-1) = toc;
    
    % qmdp
    tic
    mdp_sol = mdp_prob.solve();
    t1 = toc;
    qmdp_prob = qmdpProblem(n,H,inp.T,inp.Z,inp.R,inp.A,mdp_sol.V);
    tic
    qmdp_prob.solve();
    run_time(2,n-1) = toc + t1;
    
    % pomdp full
    pomdp_prob = pomdpProblem(n,H,inp.T,inp.Z,inp.R,inp.A,'solver','full');
    tic
    pomdp_prob.solve();
    run_time(3,n-1) = toc;
  
    % pomdp larks
    pomdp_prob = pomdpProblem(n,H,inp.T,inp.Z,inp.R,inp.A,'solver','larks');
    tic
    pomdp_prob.solve();
    run_time(4,n-1) = toc;
    
    % pomdp pbvi
    n_samp = 30;  % number of samples
    b_samp_mat = ones(n)/n^2;
    b_samp = reshape(b_samp_mat,[n^2,1]);
    pomdp_prob = pomdpProblem(n,H,inp.T,inp.Z,inp.R,inp.A,'solver','pbvi',b_samp,n_samp);
    tic
    pomdp_prob.solve();
    run_time(5,n-1) = toc;
    
    run_time
  end
  
  %% Plot
  if plot_complexity
      figure
      colors = lines(size(run_time,1));
      hold on
      
      % loop over algorithms
      for i=1:5
        plot(state_size,run_time(i,2:end),'*','color',colors(i,:),'LineWidth',2)
      end
      
      xlabel('State Size')
      ylabel('Time [s]')
      legend('MDP','QMDP','POMDP-FULL','POMDP-LARKS','POMDP-PBVI','Location','NorthEast')
      
      hold off
  end
end

%% Obstacles


%% Blind Spots


%% Initial Belief Uncertainty


%% Belief Sample Size
