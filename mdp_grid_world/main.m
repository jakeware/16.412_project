clear all
close all
clc

%% Setup
n = 20;
inp = setupProblem(n);

%% Exectuion
prob = mdpProblem(n^2,inp.T,inp.R,inp.A);
sol = prob.solve();
prob.plot_sol(sol);