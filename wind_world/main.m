clear all
close all
clc

%% Environment Setup
n = 4;  % grid size (for nxn grid)
H = 5;  % solver horizon
sim_time = 8;  % simulation steps

% get actions and compute reward, transition, and observation functions
inp = setupProblem(n);

%% Solver Selection
solve_mdp = 0;  % MDP
solve_qmdp = 0;  % QMDP
solve_pomdp = 0;  % POMDP
solve_pomdp_larks = 0;  % POMDP with larks pruning
solve_pomdp_pbvi = 1;  % POMDP with PBVI

%% MDP Inputs
s0 = [1,1];  % initial state for mdp

%% POMDP Inputs
% initial belief for simulation
b0_sim_mat = zeros(n);
b0_sim_mat(1,1) = 1;
b0_sim = reshape(b0_sim_mat,[n^2,1]);

% belief for PBVI sampling
n_samp = 30;  % number of samples

% uniform belief
b_samp_mat = ones(n)/n^2;
b_samp = reshape(b_samp_mat,[n^2,1]);

% diagonal belief
%b_samp_mat = eye(n);
%b_samp = reshape(b_samp_mat,[n^2,1]);

% at origin with no uncertainty
%b_samp_mat = zeros(n);
%b_samp_mat(1,1) = 1;
%b_samp = reshape(b_samp_mat,[n^2,1]);

% near goal

%% Execution
% mdp or qmdp
if solve_mdp || solve_qmdp
  mdp_prob = mdpProblem(n,inp.T,inp.R,inp.A);
  mdp_sol = mdp_prob.solve();
  
  % solve mdp
  if solve_mdp
    path = mdp_prob.simulate(mdp_sol,s0,H);
    mdp_prob.plot_sol(mdp_sol, path);
  end
  
  % solve qmdp
  if solve_qmdp
    qmdp_prob = qmdpProblem(n,H,inp.T,inp.Z,inp.R,inp.A,mdp_sol.V);
    qmdp_sol = qmdp_prob.solve();
    path = qmdp_prob.simulate(qmdp_sol,b0_sim);
    qmdp_prob.plot_sol(path);
  end
end

% solve pomdp
if solve_pomdp
  pomdp_prob = pomdpProblem(n,H,inp.T,inp.Z,inp.R,inp.A,'solver','full');
  pomdp_sol = pomdp_prob.solve();
  path = pomdp_prob.simulate(pomdp_sol,b0_sim,sim_time);
  pomdp_prob.plot_sol(path);
end

% solve pomdp with larks
if solve_pomdp_larks
  pomdp_prob = pomdpProblem(n,H,inp.T,inp.Z,inp.R,inp.A,'solver','larks');
  pomdp_sol = pomdp_prob.solve();
  path = pomdp_prob.simulate(pomdp_sol,b0_sim,sim_time);
  pomdp_prob.plot_sol(path);
end

% solve pomdp with PBVI
if solve_pomdp_pbvi
  pomdp_prob = pomdpProblem(n,H,inp.T,inp.Z,inp.R,inp.A,'solver','pbvi',b_samp,n_samp);
  pomdp_sol = pomdp_prob.solve();
  path = pomdp_prob.simulate(pomdp_sol,b0_sim,sim_time);
  pomdp_prob.plot_sol(path);
end