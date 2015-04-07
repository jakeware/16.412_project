clear all
close all
clc

n = 25;
prob = mdpProblem(n,@(s)Reward(n,s));
sol = prob.solve();
prob.plot(sol);