function Z = createObservationMatrix(node_list,edge_list,obs)
  %% extract goodlist/badlist from map
  map = obs;
  numstates = length(node_list); %length(node_list)

  %% find occluded ("badlist") nodes
  % "badlist" nodes are occluded nodes

  badlist = zeros(numstates,1);
  for i = 1:size(node_list,1)
      for k = 3:21
          type = map(node_list(i,2), node_list(i,1), k);
          if type == 0
              badlist(i) = 1;
              break
          end
      end
  end

  badlist_nodes = find(badlist == 1);

  %% Identify good and bad edges
  bad_edges = [];
  good_edges = [];
  for i= 1:size(edge_list,1)
      if any(ismember(badlist_nodes, edge_list(i,:)))
          % bad edges: edges connecting any node to a bad node
          bad_edges = [bad_edges; edge_list(i,:)];
      else
          % good edges: only edges connecting good nodes
          good_edges = [good_edges; edge_list(i,:)];
      end
  end


  %% Given a goodlist, assign observation probabilities based on neighbors
  % 3-2-1 ratio of correct-neighbor-other

  Z = zeros(numstates);

  % these need to be functions of how many there are
  % ignoring that and just scaling to sum to 1 at end
  other_rate = 0.1;
  good_neighbor_rate = 0.2;
  bad_neighbor_rate = 0.2;
  correct_rate = 0.3;

  % this double-counts bad edges if they are bi-directional, may want to fix

  for i = 1:numstates
      if badlist(i) == 0
          [I_good,J] = find(good_edges==i);
          [I_bad,J] = find(bad_edges==i);

          good_edges_copy = good_edges(I_good, :);
          good_edges_copy(good_edges_copy==i) = [];

          Z(i, :) = other_rate;
          Z(i, good_edges_copy) = good_neighbor_rate;
          Z(i, numstates+1) = length(I_bad)*bad_neighbor_rate;

          Z(i,i) = correct_rate;
      else
          Z(i,numstates+1) = 1;
      end
  end

  Z(:, badlist_nodes) = [];
  Y = sum(Z,2);

  for i = 1:numstates
      Z(i,:) = Z(i,:)/Y(i);
  end

end
  