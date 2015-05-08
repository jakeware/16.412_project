function [xtraj,gtraj,htraj,atraj,gmax] = Astar(start,goal,speeds,obs,wind)
    
  alpha = 0.5;
  
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
  
%   h = figure(55);
%   hold on;
%   xlim([0 100]);
%   ylim([0 100]);
%   axis equal;
  
  while (pq_size(opensetpq)>0)
    
    [current_i,min_f] = pq_pop(opensetpq);
    current = openset(:,current_i);
    in_openset(current(1),current(2),current(3),current(4)) = 0;
    
    %figure(h);
    %plot3(current(1),current(2),10*norm([current(3),current(4)]),'*r');
    %pause;
    
%     if g_score(current(1),current(2),current(3),current(4))>5000
%         % energy consumption of the current best trajectory is too high
%         break;
%     end
    
    if all(current(:)==goal(:))
      [xtraj,gtraj,htraj,atraj] = reconstruct_path(came_from_row,came_from_col,came_from_row_spe,came_from_col_spe,start,goal,speeds,g_score,wind);
      return;
    end
    
    expanded_states(current(1),current(2),current(3),current(4)) = 1;
    neighs = get_neighbours(current,n,speeds,obs,wind);
    for i=1:size(neighs,2)
      neigh = neighs(:,i);
      if expanded_states(neigh(1),neigh(2),neigh(3),neigh(4))
        continue;
      end
      tentative_g_score = g_score(current(1),current(2),current(3),current(4)) + dist_between(current,neigh,speeds,wind);
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
  
  % empty queue, didn't find goal
  xtraj = 0;
  gtraj = 0;
  htraj = 0;
  atraj = 0;
  gmax = 0;
end

function h = heuristic_cost_estimate(state,goal,speeds)
  h = norm(goal(1:2)-(state(1:2)+[speeds(state(3));speeds(state(4))]));
end

function cost = dist_between(state1,state2,speeds,wind)
%   cost = norm(state2(1:2)-state1(1:2));
  
  groundspeed = [speeds(state1(3)) speeds(state1(4)) 0];
  airspeed = get_airspeed(groundspeed,state1,wind);
  
  nominal_power = 10;
  dt = 1;
  if norm(state1(3:4))==0
      energy = dt*nominal_power;
  else
      energy = dt*(nominal_power + norm(airspeed)^2);
  end
    
  cost = energy;
end

function airspeed = get_airspeed(groundspeed,state,wind)
  airspeed = zeros(1,3);
  airspeed(1) = groundspeed(1) - wind(state(2),state(1),1);
  airspeed(2) = groundspeed(2) - wind(state(2),state(1),2);
  airspeed(3) = groundspeed(3) - wind(state(2),state(1),3);
end
 
function [xtraj,gtraj,htraj,atraj] = reconstruct_path(came_from_row,came_from_col,came_from_row_spe,came_from_col_spe,start,goal,speeds,g_score,wind)
  xtraj = goal;
  gtraj = g_score(goal(1),goal(2),goal(3),goal(4));
  htraj = heuristic_cost_estimate(goal,goal,speeds);
  atraj = norm(get_airspeed([speeds(goal(3)) speeds(goal(4)) 0],goal,wind));
  current = goal;
  while ~all(current(:)==start(:))
    current = [came_from_row(current(1),current(2),current(3),current(4));came_from_col(current(1),current(2),current(3),current(4));...
      came_from_row_spe(current(1),current(2),current(3),current(4));came_from_col_spe(current(1),current(2),current(3),current(4))];
    xtraj = [current,xtraj];
    gtraj = [g_score(current(1),current(2),current(3),current(4)),gtraj];
    htraj = [heuristic_cost_estimate(current,goal,speeds),htraj];
    atraj = [norm(get_airspeed([speeds(current(3)) speeds(current(4)) 0],current,wind)),atraj];
  end
  % converting speed indices to physicals speeds
  for j=1:size(xtraj,2)
    xtraj(3:4,j) = [speeds(xtraj(3,j));speeds(xtraj(4,j))];
  end
end

function neighs = get_neighbours(state,n,speeds,obs,wind) 
  if state(1)+speeds(state(3))>n || state(1)+speeds(state(3))<1 || state(2)+speeds(state(4))>n || state(2)+speeds(state(4))<1 || obs(state(2)+speeds(state(4)),state(1)+speeds(state(3)))==1
    neighs = zeros(4,0);
    return;
  end
      
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

      groundspeed = [speeds(newrowspeed(rowspeedi)) speeds(newcolspeed(colspeedi)) 0];
      if norm(groundspeed)>max(speeds)
          continue;
      end
      
      airspeed = get_airspeed(groundspeed,[state(1)+speeds(state(3)) state(2)+speeds(state(4))],wind);  
      if norm(airspeed)>4
       continue;
      end
      
      neighs = [neighs,[state(1)+speeds(state(3));state(2)+speeds(state(4));newrowspeed(rowspeedi);newcolspeed(colspeedi)]];
    end
  end
end