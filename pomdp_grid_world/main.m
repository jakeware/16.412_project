clear all
close all
clc

%% Notes
% This is a test of a POMDP solver on a grid world

%% Setup
n = 4;  % number of states
gamma = 0.9;  % dicount factor

% latent initial state
s0 = [1,1];

% initial belief
b0 = zeros(n);
b0(s0(1),s0(2)) = 1;

% States
% S \in {x,y}
% (1,n),...,(n,n);
% ...,...,...;
% (1,1),...,(n,1);

% Observations
% 60% observe where you are
% 5% for each surrounding location 


%% Analysis


%% Plot
