clear all
close all
clc

%% Setup
solve_qmdp = 0;
solve_pomdp_larks = 1;  % POMDP with larks pruning

%% Load Data
load('../../quic_project_v3_data/setup/mc/batchdata.mat');
load('../../quic_project_v3_data/results/energy.mat');
load('../../quic_project_v3_data/celltype.mat');

z = 2;
obs = celltype{1}(:,:,z)==0;
map = celltype{1};

%% Inflate Obstacles
n = size(obs,1);
inf_obs = zeros(n);
rad = 1;
for i=1:n
    for j=1:n
        found = 0;
        ioffset = -rad:rad;
        joffset = -rad:rad;
        for ii=1:numel(ioffset)
           for jj=1:numel(joffset)
              to_check = [i,j]+[ioffset(ii),joffset(jj)];
              if to_check(1)>n || to_check(1)<1 || to_check(2)>n || to_check(2)<1
                  continue;
              else
                 found = found | obs(to_check(1),to_check(2));
              end
           end
        end
        inf_obs(i,j) = found;
    end
end
obs = inf_obs;

%% Graph
% node locations (x, y)
node_list = [
    2,2;   % 1
    2,25;  % 2
    2,48;  % 3
    13,19; % 4
    13,37; % 5
    25,2;  % 6
    25,22; % 7
    25,48; % 8
    42,13; % 9
    42,37; % 10
    48,2;  % 11
    48,23; % 12
    48,48; % 13
];

% graph edges (node index, node index)
edge_list = [
    1,2;
    1,4;
    1,6;
    2,1;
    2,3;
    2,5;
    2,4;
    2,7;
    3,2;
    3,5;
    3,8;
    4,1;
    4,2;
    4,5;
    4,6;
    4,7;
    5,2;
    5,3;
    5,4;
    5,7;
    5,8;
    6,1;
    6,4;
    6,7;
    6,9;
    6,11;
    7,4;
    7,5;
    7,6;
    7,8;
    7,9;
    7,10;
    7,12;
    7,13;
    8,3;
    8,5;
    8,7;
    8,10;
    8,13;
    9,6;
    9,7;
    9,10;
    9,11;
    9,12;
    10,7;
    10,8;
    10,9;
    10,12;
    10,13;
    11,6;
    11,9;
    11,12;
    12,7;
    12,9;
    12,10;
    ];

goal = 13;

%% Wind Distribution
uv_mean = [-4, 4]';  % x-vel [m/s]
uv_cov = [
    10, -3;
    -3, 5];

%% Environment Setup
H = 5;  % solver horizon
sim_time = 8;  % simulation steps

% get actions and compute reward, transition, and observation functions
[T,R,Z] = setupProblem(node_list,edge_list,goal,uv_mean,uv_cov,batchdata,energy,z,map);

%% MDP Inputs
s0 = 1;  % initial state for mdp

%% POMDP Inputs
% initial belief for simulation
b0_sim_mat = zeros(size(T,1),1);
b0_sim_mat(1,1) = 1;

%% Execution
% mdp or qmdp
if solve_qmdp
  mdp_prob = mdpProblem(T,R);
  mdp_sol = mdp_prob.solve();
  qmdp_prob = qmdpProblem(H,T,Z,R,mdp_sol.V);
  qmdp_sol = qmdp_prob.solve();
  %path = qmdp_prob.simulate(qmdp_sol,b0_sim_mat,sim_time);
  %qmdp_prob.plot_sol(path);
end

% solve pomdp with larks
if solve_pomdp_larks
  pomdp_prob = pomdpProblem(H,T,Z,R,'solver','larks');
  pomdp_sol = pomdp_prob.solve();
  path = pomdp_prob.simulate(pomdp_sol,b0_sim_mat,sim_time);
  pomdp_prob.plot_sol(path,sim_time,node_list,obs);
end