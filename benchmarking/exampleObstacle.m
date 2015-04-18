clear all
close all
clc

%% Setup
mdp = 0;
qmdp = 0;
pomdp = 1;

n = 4;
H = 8;  % solver horizon
sim_time = 12;  % simulation steps

% get actions and compute reward, transition, and observation functions
inp = setupObstacle(n);

%% Inputs
s0 = [1,1];  % initial state for mdp

% initial belief for simulation
b0_sim_mat = zeros(n);
b0_sim_mat(1,1) = 1;
b0_sim = reshape(b0_sim_mat,[n^2,1]);

%% Execution
if mdp || qmdp
    % mdp solution
    mdp_prob = mdpProblem(n,inp.T,inp.R,inp.A);
    mdp_sol = mdp_prob.solve();
    
    % mdp simulation
    if mdp
        path = mdp_prob.simulate(mdp_sol,s0,sim_time);
        mdp_prob.plot_sol(mdp_sol, path);
    end

    % qmdp
    if qmdp
        qmdp_prob = qmdpProblem(n,H,inp.T,inp.Z,inp.R,inp.A,mdp_sol.V);
        qmdp_sol = qmdp_prob.solve();
        path = qmdp_prob.simulate(qmdp_sol,b0_sim,sim_time);
        qmdp_prob.plot_sol(path,sim_time);
    end
end

% pomdp larks
if pomdp
    pomdp_prob = pomdpProblem(n,H,inp.T,inp.Z,inp.R,inp.A,'solver','larks');
    pomdp_sol = pomdp_prob.solve();
    path = pomdp_prob.simulate(pomdp_sol,b0_sim,sim_time);
    pomdp_prob.plot_sol(path,sim_time);
end