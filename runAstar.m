load('celltype.mat')
obs = celltype{1}(:,:,2)==0;
n = size(obs,1);
speeds = [0 1 2];
start = [82 22 0 0];
goal = [90 22 0 0];
%goal = [175 160 0 0];
traj = Astar(start,goal,n,speeds,obs);
hold on
imagesc(obs);
plot(traj(1,:),traj(2,:),'r*');
xlim([0 n]);
ylim([0 n]);
axis equal;