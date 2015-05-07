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
    50,150;
    190,180;
    10,180];

% graph edges (node index, node index)
edge_list = [
    1,2;
    2,1];  

% store total energy and binary success for each traj
energy = zeros(size(listing,1),size(edge_list,1));
success = zeros(size(listing,1),size(edge_list,1));

%% Load Data
% load obs
load(strcat(data_path,'celltype.mat'));
obs = celltype{1}(:,:,2)==0;

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
    vel = load(strcat(data_path,listing(i).name,'/velocity.mat'));
    
    for j=1:size(edge_list,1)
        fprintf('start_node: %i, goal_node: %i\n', edge_list(j,1), edge_list(j,2))
        start_state = [node_list(edge_list(j,1),:),0,0];
        goal_state = [node_list(edge_list(j,2),:),0,0];
        
        %[energy(i,j),success(i,j)] = astar(start_state,goal_state,obs,vel);
    end
end

% create results dir
if ~exist(strcat(data_path,'results'), 'dir')
  mkdir(strcat(data_path,'results'));
end

% save results
save(strcat(data_path,'results/energy.mat'),'energy');
save(strcat(data_path,'results/success.mat'),'success');