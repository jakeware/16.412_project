clear all
close all
clc

%% Setup
n = 2;  % grid size
H = 5;  % horizon

solve_mdp = 0;  % MDP
solve_qmdp = 1;  % QMDP
solve_pomdp = 0;  % POMDP

% initial conditions
s0 = [1,1];
b0_mat = zeros(n);
b0_mat(1,1) = 1;
b0 = reshape(b0_mat,[n^2,1]);

inp = setupProblem(n);

%% Exectuion
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
    path = qmdp_prob.simulate(qmdp_sol,b0);
    qmdp_prob.plot_sol(path);
  end
end

% solve pomdp
if solve_pomdp
  pomdp_prob = pomdpProblem(n,H,inp.T,inp.Z,inp.R,inp.A);
  pomdp_sol = pomdp_prob.solve();
end