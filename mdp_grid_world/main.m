clear all
close all
clc

prob = mdpProblem(4,@Reward);
sol = prob.solve();
prob.plot(sol);