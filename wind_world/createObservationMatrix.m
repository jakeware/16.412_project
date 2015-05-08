%goodlist = (rand(numstates,1)>0.3)
clc


goodlist = [1 0 0 1 0 1 1];
edge_list = [1 2; 1 4; 2 3; 3 4; 3 5; 4 5; 4 6; 6 7];
numstates = length(goodlist); %length(node_list)


% good edge list: only edges connecting good nodes
badlist = not(goodlist);
badlist_nodes = find(badlist == 1);

bad_edges = [];
good_edges = [];
for i= 1:size(edge_list,1)
    if any(ismember(badlist_nodes, edge_list(i,:)))
        bad_edges = [bad_edges; edge_list(i,:)];
    else
        good_edges = [good_edges; edge_list(i,:)];
    end
end


%% Given a goodlist, assign probabilities based on neighbors

% 3-2-1 ratio of correct-neighbor-other

Z = zeros(numstates);

% these need to be functions of how many there are
% ignoring that and just scaling to sum to 1 at end
other_rate = 0.1;
good_neighbor_rate = 0.2;
bad_neighbor_rate = 0.2;
correct_rate = 0.3;

for i = 1:numstates
    if goodlist(i) == 1
        display('node is good')
        display(i)
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

Z
  