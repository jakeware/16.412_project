function traj = Astar(start,goal,n,obs)
	closedset = [];
	openset = [start'];
	came_from_col = -1*ones(n);
  came_from_row = -1*ones(n);

	g_score = Inf*ones(n);
  g_score(start(1),start(2)) = 0;
	f_score = Inf*ones(n);
  f_score(start(1),start(2)) = g_score(start(1),start(2)) + heuristic_cost_estimate(start,goal);

  while (size(openset,2)>0)
    current = openset(:,1);
    current_i = 1;
    min_f = f_score(current(1),current(2));
    for i=2:size(openset,2);
      current_candidate = openset(:,i);
      if f_score(current_candidate(1),current_candidate(2))<min_f
        min_f = f_score(current_candidate(1),current_candidate(2));
        current = current_candidate;
        current_i = i;
      end
    end
    if all(current(:)==goal(:))
      traj = reconstruct_path(came_from_row,came_from_col,start,goal);
      return;
    end
    
    openset(:,current_i) = [];
    closedset = [closedset,current];
    neighs = get_neighbours(current,n,obs);
    for i=1:size(neighs,2)
      neigh = neighs(:,i);
      if ismember(neigh',closedset','rows')
        continue;
      end
      
      tentative_g_score = g_score(current(1),current(2)) + dist_between(current,neigh);
      
      if ~ismember(neigh',openset','rows') || tentative_g_score < g_score(neigh(1),neigh(2))
        came_from_row(neigh(1),neigh(2)) = current(1);
        came_from_col(neigh(1),neigh(2)) = current(2);
        g_score(neigh(1),neigh(2)) = tentative_g_score;
        f_score(neigh(1),neigh(2)) = g_score(neigh(1),neigh(2)) + heuristic_cost_estimate(neigh,goal);
        if ~ismember(neigh',openset','rows')
          openset = [openset,neigh];
        end
      end
    end
    
  end
  
  traj = 0;
end

function cost = heuristic_cost_estimate(state,goal)
  cost = sum(abs(goal(:)-state(:)));
end

function dist = dist_between(state1,state2)
  dist = sum(abs(state2(:)-state1(:)));
end

function traj = reconstruct_path(came_from_row,came_from_col,start,goal)
  traj = [goal'];
  current = goal';
  while ~all(current(:)==start(:))
    current = [came_from_row(current(1),current(2));came_from_col(current(1),current(2))];
    traj = [current,traj];
  end
end

function clean_neighs = get_neighbours(state,n,obs)
  if state(1)==1&&state(2)==1
    neighs = [[1;2],[2;1]];
  elseif state(1)==1&&state(2)==n
    neighs = [[1;n-1],[2;n]];
  elseif state(1)==n&&state(2)==1
    neighs = [[n-1;1],[n;2]];
  elseif state(1)==n&&state(2)==n
    neighs = [[n-1;n],[n;n-1]];
  elseif state(1)==1
    neighs = [[1;state(2)-1],[2;state(2)],[1;state(2)+1]];
  elseif state(1)==n
    neighs = [[n;state(2)-1],[n-1;state(2)],[n;state(2)+1]];
  elseif state(2)==1
    neighs = [[state(1)-1;1],[state(1);2],[state(1)+1;1]];
  elseif state(2)==n
    neighs = [[state(1)-1;n],[state(1);n-1],[state(1)+1;n]];
  else
    neighs = [[state(1)+1;state(2)],[state(1)-1;state(2)],[state(1);state(2)-1],[state(1);state(2)+1]];
  end
  
  clean_neighs = [];
  for i=1:size(neighs,2)
    if obs(neighs(2,i),neighs(1,i))==0
      clean_neighs = [clean_neighs,neighs(:,i)];
    end
  end

end