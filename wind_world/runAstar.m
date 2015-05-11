clear all
close all
clc

z = 2;

addpath('../queue');

load('../../quic_project_v3_data/celltype.mat')
obs = celltype{1}(:,:,z)==0;
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

load('../../quic_project_v3_data/quic_project_v3_samp005/velocity.mat')
wind(:,:,1) = velocity.u{1}(:,:,z);
wind(:,:,2) = velocity.v{1}(:,:,z);
wind(:,:,3) = velocity.w{1}(:,:,z);

speeds = [-3 -2 -1 0 1 2 3];
%start = [80 22 0 0]';
start = [13 19 0 0]';
goal = [13 37 0 0]';
% start = [15 75 0 0]';
% goal = [25 10 0 0]';

figure(74);
hold on;
imagesc(obs);
plot(start(1),start(2),'bo');
plot(goal(1),goal(2),'ro');

if obs(start(2),start(1)) == 1 || obs(goal(2),goal(1)) == 1
    error('Start or goal in obstacle region.')
end

display('running A*...');
tic;
[xtraj,gtraj,htraj,atraj] = Astar(start,goal,speeds,obs,wind);
toc
display('done.');
if xtraj==0
    display('No trajectory found');
    return;
end

figure(75);
hold on;
vtraj = sqrt(sum(xtraj(3:4,:).^2,1));
imagesc(velocity.mag{1}(:,:,z));
scatter(xtraj(1,:),xtraj(2,:),25,vtraj,'filled');
scatter(start(1),start(2),100,'g','filled');
scatter(goal(1),goal(2),100,'r','filled');
title('Minimum-Energy Trajectory in Sample Wind Field')
xlabel('X [m]')
ylabel('Y [m]')
xlim([0 n]);
ylim([0 n]);
hold off

figure(76);
hold on
plot(vtraj,'b.-');
plot(atraj,'or-');
title('Air and Ground Speed Vs. Time');
xlabel('Time [s]')
ylabel('Velocity [m/s]')
legend('Vg','Va')
hold off

figure(77);
hold on
plot(1:numel(gtraj),gtraj,1:numel(htraj),htraj,'*');
title('Planner Cost Vs. Steps')
ylabel('Cost')
xlabel('Steps')
legend('G', 'H');
hold off