load('celltype.mat')
obs = celltype{1}(:,:,2)==0;

n = size(obs,1);
speeds = [-1 0 1];
start = [82 22 0 0];
goal = [90 75 0 0];
%goal = [82 30 0 0];

figure(75);
hold on;
imagesc(obs);
plot(start(1),start(2),'bo');
plot(goal(1),goal(2),'ro');

display('running A*...');
traj = Astar(start,goal,n,speeds,obs);
display('done.');

figure(75);
hold on;
vtraj = sqrt(sum(traj(3:4,:).^2,1));
imagesc(obs*max(vtraj)*1.5);
scatter(traj(1,:),traj(2,:),25,vtraj,'filled');
xlim([0 n]);
ylim([0 n]);
axis equal;

figure(76);
plot(vtraj,'b.-');
ylim([min(speeds) max(speeds)]);