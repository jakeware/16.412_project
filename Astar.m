function xtraj = Astar(start,goal,n)
	closedset = [];
	openset = [start'];
	came_from = [];

	g_score = Inf*ones(n,n);
  g_score(start(1),start(2)) = 0;
	f_score = Inf*ones(n,n);
  f_score(start(1),start(2)) = heuristic_cost_estimate(start,goal);

  while (numel(openset)>0)
    current = openset(:,1);
    current_i = 1;
    max_f = f_score(current(1),current(2));
    for i=2:size(openset,2);
      current_candidate = openset(:,i);
      if f_score(current_candidate(1),current_candidate(2))>max_f
        max_f = f_score(current_candidate(1),current_candidate(2));
        current = current_candidate;
        current_i = i;
      end
    end
    if all(current==goal)
      reconstruct_path(came_from,goal);
    end
    
    openset(:,current_i) = [];
    closedset = [closedset,current];
    neighs = get_neighbours(current)
    for i=1:size(neighs,2)
      neigh = neighs(:,i);
      if all(ismember(neigh,closedset))
        continue;
      end
      
      tentative_g_score = g_score(current(1),current(2)) + dist_between(current,neigh);
      
      if ~any(ismember(neigh,openset)) || tentative_g_score < g_score(neigh(1),neigh(2))
        came_from_col(neigh(1),neigh(2)) = current(1);
        came_from_row(neigh(1),neigh(2)) = current(2);
        g_score(neigh(1),neigh(2)) = tentative_g_score;
        f_score(neigh(1),neigh(2)) = g_score(neigh(1),neigh(2)) + heurisitc_cost_estimate(neigh,goal);
        if ~any(ismember(neigh,openset))
          openset = [openset,neigh];
        end
      end
    end
    
  end
  
  xtraj = 0;
end

function cost = heuristic_cost_estimate(start,goal)
  cost = norm(goal-start);
end

function traj = reconstruct_path(came_from,current)
  
end

function neighs = get_neighbours(state)

end

function dist = dist_between(state1,state2)
  dist = norm(state2,state1);
end