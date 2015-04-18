clear all
close all
clc

n = 2;
H = [5,10,15];  % solver horizon

% result vectors
run_time = zeros(5,1:length(H));

% iteratore over grid size
for i=1:length(H)
  i

  % get actions and compute reward, transition, and observation functions
  inp = setupComplexity(n);

  % mdp
  mdp_prob = mdpProblem(n,inp.T,inp.R,inp.A);
  tic
  mdp_sol = mdp_prob.solve();
  run_time(1,i) = toc;

  % qmdp
  qmdp_prob = qmdpProblem(n,H(i),inp.T,inp.Z,inp.R,inp.A,mdp_sol.V);
  tic
  qmdp_prob.solve();
  run_time(2,i) = toc + run_time(1,n-1);

  % pomdp full
  pomdp_prob = pomdpProblem(n,H(i),inp.T,inp.Z,inp.R,inp.A,'solver','full');
  tic
  pomdp_prob.solve();
  run_time(3,i) = toc;

  % pomdp larks
  pomdp_prob = pomdpProblem(n,H(i),inp.T,inp.Z,inp.R,inp.A,'solver','larks');
  tic
  pomdp_prob.solve();
  run_time(4,i) = toc;

  % pomdp pbvi
  n_samp = 30;  % number of samples
  b_samp_mat = ones(n)/n^2;
  b_samp = reshape(b_samp_mat,[n^2,1]);
  pomdp_prob = pomdpProblem(n,H(i),inp.T,inp.Z,inp.R,inp.A,'solver','pbvi',b_samp,n_samp);
  tic
  pomdp_prob.solve();
  run_time(5,i) = toc;

  run_time
end
  
%% Plot
% plot
figure
colors = lines(size(run_time,1));
hold on

% loop over algorithms
for i=1:5
  plot(H,run_time(i,:),'-*','color',colors(i,:),'LineWidth',2)
end

title('Runtime Vs State Size')
xlabel('State Size')
ylabel('Time [s]')
legend('MDP','QMDP','POMDP-FULL','POMDP-LARKS','POMDP-PBVI','Location','NorthWest')
%axis([3,10,0.5,10])

hold off

% plot
figure
colors = lines(size(run_time,1));
hold on

% loop over algorithms
for i=1:5
  plot(H,run_time(i,:)/run_time(i,2),'-*','color',colors(i,:),'LineWidth',2)
end

title('Normalized Runtime Vs. State Size')
xlabel('State Size')
ylabel('Normalized Runtime')
legend('MDP','QMDP','POMDP-FULL','POMDP-LARKS','POMDP-PBVI','Location','NorthWest')
%axis([3,10,0.5,10])

hold off
