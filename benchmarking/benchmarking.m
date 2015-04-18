clear all
close all
clc

%% Setup
run_n_comp = 1;
run_h_comp = 0;
plot_complexity = 1;

%% Time Complexity
if run_n_comp
  n_max = 6;
  H = 6;  % solver horizon
  sim_time = 12;  % simulation steps

  % result vectors
  state_size = (2:n_max).^2;
  run_time = zeros(5,1:n_max-1);

  % iteratore over grid size
  for n=2:n_max
    n
    
    % get actions and compute reward, transition, and observation functions
    inp = setupComplexity(n);

    % mdp
    mdp_prob = mdpProblem(n,inp.T,inp.R,inp.A);
    tic
    mdp_sol = mdp_prob.solve();
    run_time(1,n-1) = toc;

    % qmdp
    qmdp_prob = qmdpProblem(n,H,inp.T,inp.Z,inp.R,inp.A,mdp_sol.V);
    tic
    qmdp_prob.solve();
    run_time(2,n-1) = toc + run_time(1,n-1);

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
  
  % plot
  figure
  colors = lines(size(run_time,1));
  hold on

  % loop over algorithms
  for i=1:5
    plot(state_size,run_time(i,:)/run_time(i,2),'-*','color',colors(i,:),'LineWidth',2)
  end

  xlabel('State Size')
  ylabel('Time [s]')
  legend('MDP','QMDP','POMDP-FULL','POMDP-LARKS','POMDP-PBVI','Location','NorthWest')
  axis([3,10,0.5,10])

  hold off
end

%% Obstacles


%% Blind Spots


%% Initial Belief Uncertainty


%% Belief Sample Size
