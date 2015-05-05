function traj = Astar(start,goal,n,speeds,obs)

  alpha = 0;

  start(3) = find(speeds==start(3));
  start(4) = find(speeds==start(4));
  goal(3) = find(speeds==goal(3));
  goal(4) = find(speeds==goal(4));
	s = numel(speeds);
  
  closedset = [];
	openset = [start'];
  came_from_col = -1*ones(n,n,s,s);
  came_from_row = -1*ones(n,n,s,s);
  came_from_col_spe = -1*ones(n,n,s,s);
  came_from_row_spe = -1*ones(n,n,s,s);
  
	g_score = Inf*ones(n,n,s,s);
  g_score(start(1),start(2),start(3),start(4)) = 0;
	f_score = Inf*ones(n,n,s,s);
  f_score(start(1),start(2),start(3),start(4)) = g_score(start(1),start(2),start(3),start(4)) + heuristic_cost_estimate(start,goal,speeds);

  figure(55);
  hold on;
  xlim([0 100]);
  ylim([0 100]);
  axis equal;
  while (size(openset,2)>0)
    current = openset(:,1);
    current_i = 1;
    min_f = f_score(current(1),current(2),current(3),current(4));
    for i=2:size(openset,2);
      current_candidate = openset(:,i);
      if f_score(current_candidate(1),current_candidate(2),current_candidate(3),current_candidate(4))<min_f
        min_f = f_score(current_candidate(1),current_candidate(2),current_candidate(3),current_candidate(4));
        current = current_candidate;
        current_i = i;
      end
    end
    if all(current(:)==goal(:))
      traj = reconstruct_path(came_from_row,came_from_col,came_from_col_spe,came_from_row_spe,start,goal,speeds);
      return;
    end
    
    %plot3(current(1),current(2),10*norm([current(3),current(4)]),'*r');
    %pause;
    
    openset(:,current_i) = [];
    closedset = [closedset,current];
    neighs = get_neighbours(current,n,speeds,obs);
    for i=1:size(neighs,2)
      neigh = neighs(:,i);
      if ismember(neigh',closedset','rows')
        continue;
      end
      
      tentative_g_score = g_score(current(1),current(2),current(3),current(4)) + dist_between(current,neigh,speeds);
      
      if ~ismember(neigh',openset','rows') || tentative_g_score < g_score(neigh(1),neigh(2),neigh(3),neigh(4))
        came_from_row(neigh(1),neigh(2),neigh(3),neigh(4)) = current(1);
        came_from_col(neigh(1),neigh(2),neigh(3),neigh(4)) = current(2);
        came_from_row_spe(neigh(1),neigh(2),neigh(3),neigh(4)) = current(3);
        came_from_col_spe(neigh(2),neigh(2),neigh(3),neigh(4)) = current(4);
        g_score(neigh(1),neigh(2),neigh(3),neigh(4)) = tentative_g_score;
        f_score(neigh(1),neigh(2),neigh(3),neigh(4)) = alpha*g_score(neigh(1),neigh(2),neigh(3),neigh(4)) + (1-alpha)*heuristic_cost_estimate(neigh,goal,speeds);
        if ~ismember(neigh',openset','rows')
          openset = [openset,neigh];
        end
      end
    end
    
  end
  
  traj = 0;
end

function cost = heuristic_cost_estimate(state,goal,speeds)
  cost = norm([goal(1) goal(2) speeds(goal(3)) speeds(goal(4))]-[state(1) state(2) speeds(state(3)) speeds(state(4))]);
end

function dist = dist_between(state1,state2,speeds)
  dist = norm([state2(1) state2(2) speeds(state2(3)) speeds(state2(4))]-[state1(1) state1(2) speeds(state1(3)) speeds(state1(4))]);
end

function traj = reconstruct_path(came_from_row,came_from_col,came_from_row_spe,came_from_col_spe,start,goal,speeds)
  traj = [goal'];
  current = goal';
  while ~all(current(:)==start(:))
    current = [came_from_row(current(1),current(2),current(3),current(4));came_from_col(current(1),current(2),current(3),current(4));...
      speeds(came_from_row_spe(current(1),current(2),current(3),current(4)));speeds(came_from_col_spe(current(1),current(2),current(3),current(4)))];
    traj = [current,traj];
  end
end

function clean_neighs = get_neighbours(state,n,speeds,obs)

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
      neighs = [neighs,[state(1)+speeds(state(3));state(2)+speeds(state(4));newrowspeed(rowspeedi);newcolspeed(colspeedi)]];
    end
  end
  clean_neighs = [];
  for i=1:size(neighs,2)
    if neighs(1,i)<=n && neighs(1,i)>=1 && neighs(2,i)<=n && neighs(2,i)>=1 && obs(neighs(2,i),neighs(1,i))==0
      clean_neighs = [clean_neighs,neighs(:,i)];
    end
  end
end