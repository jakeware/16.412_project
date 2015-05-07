clear all
close all
clc

%% Setup
data_path = '../test_site_v1_data/';
proj_name = 'quic_project_v1_samp*';
listing = dir(strcat(data_path,proj_name));

% node locations (x, y)
node_list = [
    30,20;
    100,100;
    195,40;
    195,180;
    10,180];

% graph edges (node index, node index)
edge_list = [
    1,2;
    1,3;
    1,5;
    2,3;
    2,4;
    3,2;
    3,4;
    5,2;
    5,4;
    ];

% speed list (nodes/step)
speeds = [-3 -2 -1 0 1 2 3];

% altitude (z index)
z_ind = 2;

% store total energy
energy = zeros(size(listing,1),size(edge_list,1));

% add priority queue folder
addpath('queue');

%% Load Data
% load obs
load(strcat(data_path,'celltype.mat'));
obs = celltype{1}(:,:,z_ind)==0;
n = size(obs,1);

%% Inflate Obstacles
inf_obs = zeros(n);
rad = 2;
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

%% Plot Map and Graph
figure
hold on

% obstacle region
colormap(flipud(colormap('gray')))
imagesc(obs);

% nodes
scatter(node_list(:,1),node_list(:,2),200,'b')
labels = cellstr(num2str([1:size(node_list,1)]'));
text(node_list(:,1),node_list(:,2),labels)

% plot graph (TODO: Add graph edges)
xlim([0 size(obs,1)]);
ylim([0 size(obs,1)]);
axis equal;
xlabel('X')
ylabel('Y')
hold off

display('Hit any key to begin execution...')
pause();

%% Execution
% iterate over folders
for i=1:size(listing,1)
    display(listing(i).name)
    
    % load velocity
    load(strcat(data_path,listing(i).name,'/velocity.mat'));
    wind(:,:,1) = velocity.u{1}(:,:,z_ind);
    wind(:,:,2) = velocity.v{1}(:,:,z_ind);
    wind(:,:,3) = velocity.w{1}(:,:,z_ind);
    
    for j=1:size(edge_list,1)
        fprintf('start_node: %i, goal_node: %i\n', edge_list(j,1), edge_list(j,2))
        start_state = [node_list(edge_list(j,1),:),0,0];
        goal_state = [node_list(edge_list(j,2),:),0,0];
        
        if obs(start_state(2),start_state(1)) == 1 || obs(goal_state(2),goal_state(1)) == 1
            error('Start or goal in obstacle region.')
        end
        
        [xtraj,gtraj,htraj,atraj] = Astar(start_state',goal_state',speeds,obs,wind);
        energy(i,j) = gtraj(end);
        
        if xtraj==0
            display('No trajectory found');
        end
    end
end

% create results dir
if ~exist(strcat(data_path,'results'), 'dir')
  mkdir(strcat(data_path,'results'));
end

% save results
save(strcat(data_path,'results/energy.mat'),'energy');