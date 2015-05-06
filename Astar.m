function [xtraj,gtraj,htraj] = Astar(start,goal,speeds,obs,wind)

  alpha = .5;

  n = size(obs,1);
  
  start(3) = find(speeds==start(3));
  start(4) = find(speeds==start(4));
  goal(3) = find(speeds==goal(3));
  goal(4) = find(speeds==goal(4));
  s = numel(speeds);
  
  expanded_states = zeros(n,n,s,s);
  came_from_row = -1*ones(n,n,s,s);
  came_from_col = -1*ones(n,n,s,s);
  came_from_row_spe = -1*ones(n,n,s,s);
  came_from_col_spe = -1*ones(n,n,s,s);
  
  g_score = Inf*ones(n,n,s,s);
  g_score(start(1),start(2),start(3),start(4)) = 0;
  f_score = Inf*ones(n,n,s,s);
  f_score(start(1),start(2),start(3),start(4)) = alpha*g_score(start(1),start(2),start(3),start(4)) + (1-alpha)*heuristic_cost_estimate(start,goal,speeds);
  openset = start;
  opensetpq = pq_create(n*n*s*s);
  in_openset = zeros(n,n,s,s);
  in_openset(start(1),start(2),start(3),start(4)) = 1;
  pq_push(opensetpq,1,-f_score(start(1),start(2),start(3),start(4)));
  
  h = figure(55);
  hold on;
  xlim([0 100]);
  ylim([0 100]);
  axis equal;
  
  while (size(openset,2)>0)
    
    %current = openset(:,1);
    %current_i = 1;
    %min_f = f_score(current(1),current(2),current(3),current(4));
    %for i=2:size(openset,2);
    %  current_candidate = openset(:,i);
    %  if f_score(current_candidate(1),current_candidate(2),current_candidate(3),current_candidate(4))<min_f
    %    min_f = f_score(current_candidate(1),current_candidate(2),current_candidate(3),current_candidate(4));
    %    current = current_candidate;
    %    current_i = i;
    %  end
    %end
    [current_i,min_f] = pq_pop(opensetpq);
    current = openset(:,current_i);
    in_openset(current(1),current(2),current(3),current(4)) = 0;
    
    %figure(h);
    %plot3(current(1),current(2),10*norm([current(3),current(4)]),'*r');
    %pause;
    
    if all(current(:)==goal(:))
      [xtraj,gtraj,htraj] = reconstruct_path(came_from_row,came_from_col,came_from_row_spe,came_from_col_spe,start,goal,speeds,g_score);
      return;
    end
    
    expanded_states(current(1),current(2),current(3),current(4)) = 1;
    neighs = get_neighbours(current,n,speeds,obs,wind);
    for i=1:size(neighs,2)
      neigh = neighs(:,i);
      if expanded_states(neigh(1),neigh(2),neigh(3),neigh(4))
        continue;
      end
      tentative_g_score = g_score(current(1),current(2),current(3),current(4)) + dist_between(current,neigh,speeds);
      if ~in_openset(neigh(1),neigh(2),neigh(3),neigh(4)) || tentative_g_score < g_score(neigh(1),neigh(2),neigh(3),neigh(4))
        came_from_row(neigh(1),neigh(2),neigh(3),neigh(4)) = current(1);
        came_from_col(neigh(1),neigh(2),neigh(3),neigh(4)) = current(2);
        came_from_row_spe(neigh(1),neigh(2),neigh(3),neigh(4)) = current(3);
        came_from_col_spe(neigh(1),neigh(2),neigh(3),neigh(4)) = current(4);
        g_score(neigh(1),neigh(2),neigh(3),neigh(4)) = tentative_g_score;
        f_score(neigh(1),neigh(2),neigh(3),neigh(4)) = alpha*g_score(neigh(1),neigh(2),neigh(3),neigh(4)) + (1-alpha)*heuristic_cost_estimate(neigh,goal,speeds);
        if ~in_openset(neigh(1),neigh(2),neigh(3),neigh(4))
          openset = [openset,neigh];
          in_openset(neigh(1),neigh(2),neigh(3),neigh(4)) = 1;
          pq_push(opensetpq,size(openset,2),-f_score(neigh(1),neigh(2),neigh(3),neigh(4)));
        end
      end
    end
  end
  
  xtraj = 0;
  gtraj = 0;
  htraj = 0;
end

function h = heuristic_cost_estimate(state,goal,speeds)
  h = norm(goal(1:2)-(state(1:2)+[speeds(state(3));speeds(state(4))]));
end

function cost = dist_between(state1,state2,speeds)
  cost = norm(state2(1:2)-state1(1:2));
end

function [xtraj,gtraj,htraj] = reconstruct_path(came_from_row,came_from_col,came_from_row_spe,came_from_col_spe,start,goal,speeds,g_score)
  xtraj = [goal];
  gtraj = g_score(goal(1),goal(2),goal(3),goal(4));
  htraj = heuristic_cost_estimate(goal,goal,speeds);
  current = goal;
  while ~all(current(:)==start(:))
    current = [came_from_row(current(1),current(2),current(3),current(4));came_from_col(current(1),current(2),current(3),current(4));...
      came_from_row_spe(current(1),current(2),current(3),current(4));came_from_col_spe(current(1),current(2),current(3),current(4))];
    xtraj = [current,xtraj];
    gtraj = [g_score(current(1),current(2),current(3),current(4)),gtraj];
    htraj = [heuristic_cost_estimate(current,goal,speeds),htraj];
  end
  % converting speed indices to physicals speeds
  for j=1:size(xtraj,2)
    xtraj(3:4,j) = [speeds(xtraj(3,j));speeds(xtraj(4,j))];
  end
end

function clean_neighs = get_neighbours(state,n,speeds,obs,wind)
  if state(3)==1
    newrowspeed = 1:2;
  elseif state(3)==numel(speeds)
    newrowspeed = numel(speeds)-1:numel(speeds);
  else
    newrowspeed = state(3)-1:state(3)+1;
  end
  if state(4)==1
    newcolspeed = 1:2;
  elseif state(4)==numel(speeds)
    newcolspeed = numel(speeds)-1:numel(speeds);
  else
    newcolspeed = state(4)-1:state(4)+1;
  end
  neighs = [];
  for rowspeedi=1:numel(newrowspeed)
    for colspeedi=1:numel(newcolspeed)

      if state(1)+speeds(state(3))>n || state(1)+speeds(state(3))<1 || state(2)+speeds(state(4))>n || state(2)+speeds(state(4))<1 || obs(state(1)+speeds(state(3)),state(2)+speeds(state(4)))==1
          continue;
      end
      
      groundspeed = [speeds(newrowspeed(rowspeedi)) speeds(newcolspeed(colspeedi))];

      if norm(groundspeed)>max(speeds)
          continue;
      end
      airspeed = zeros(1,3);
      airspeed(1) = groundspeed(1) - wind(state(1)+speeds(state(3)),state(2)+speeds(state(4)),1);
      airspeed(2) = groundspeed(2) - wind(state(1)+speeds(state(3)),state(2)+speeds(state(4)),2);
      airspeed(3) = -wind(state(1)+speeds(state(3)),state(2)+speeds(state(4)),3);
      if norm(airspeed)>3
        continue
      end
      neighs = [neighs,[state(1)+speeds(state(3));state(2)+speeds(state(4));newrowspeed(rowspeedi);newcolspeed(colspeedi)]];
    end
  end
  
  %clean_neighs = [];
  %for i=1:size(neighs,2)
  %  if neighs(1,i)<=n && neighs(1,i)>=1 && neighs(2,i)<=n && neighs(2,i)>=1 && obs(neighs(2,i),neighs(1,i))==0
  %    clean_neighs = [clean_neighs,neighs(:,i)];
  %  end
  %end
  
end