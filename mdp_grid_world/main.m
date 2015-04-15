clear all
close all
clc

%% Setup

%% Exectuion
n = 25;
prob = mdpProblem(n,@(s)Reward(n,s));
sol = prob.solve();
prob.plot(sol);