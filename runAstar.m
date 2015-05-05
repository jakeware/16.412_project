load('celltype.mat')
obs = celltype{1}(:,:,2)==0;
n = size(obs,1);
start = [82 22];
goal = [175 160];
traj = Astar(start,goal,n,obs);
hold on
imagesc(obs);
plot(traj(1,:),traj(2,:),'r*');
xlim([0 n]);
ylim([0 n]);
axis equal;