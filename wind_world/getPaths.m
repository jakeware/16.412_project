clear all
close all
clc

%% Setup
% 200x200 map
%data_path = '../../quic_project_v1_data/';
%proj_name = 'quic_project_v1_samp*';

% 50x50 map
data_path = '../../quic_project_v2_data/';
proj_name = 'quic_project_v2_samp*';

listing = dir(strcat(data_path,proj_name));

% node locations (x, y)
node_list = [
    2,2;
    25,2;
    48,2;
    13,13;
    13,37;
    25,25;
    25,48;
    2,25;
    2,48;
    48,25;
    48,48;
    42,37;
    42,13];

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
addpath('../queue');

%% Load Data
% load obs
load(strcat(data_path,'celltype.mat'));
obs = celltype{1}(:,:,z_ind)==0;
n = size(obs,1);

%% Inflate Obstacles
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

%% Plot Map and Graph
figure
hold on

% obstacle region
colormap(flipud(colormap('gray')))
imagesc(obs);

% plot nodes
scatter(node_list(:,1),node_list(:,2),200,'b','LineWidth',2)

% plot node labels
labels = cellstr(num2str([1:size(node_list,1)]'));
text(node_list(:,1),node_list(:,2),labels)

% plot edges
for i=1:length(edge_list)
    nx = node_list(edge_list(i,1),1);
    ny = node_list(edge_list(i,1),2);
    vx = node_list(edge_list(i,2),1) - nx;
    vy = node_list(edge_list(i,2),2) - ny;
    offset = 2*[vx,vy]/norm([vx,vy]);
    quiver(nx+offset(1),ny+offset(2),vx,vy,'r','LineWidth',2)
end

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