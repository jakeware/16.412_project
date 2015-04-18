%% Summary
% This function takes the grid size and computes the transition, reward,
% and observation functions for a nxn grid world.  The action set is also defined in this file
% and must be created or modified by the user.

%% Inputs
% n: the grid dimension as a real number.

%% Outputs
% A: action set (m x 2)
% T: transition function (n^2 x m x n^2)
% R: reward function (n^2 x m x n^2)
% Z: observation function (n^2 x n^2)

function sol = setupObstacle(n)
  % Actions
  % up, right, down, left, stay
  A = [
    -1,0;  % decrement row (up)
    0,1;  % increment column (right)
    1,0;  % increment row (down)
    0,-1;  % decrement column (left)
    0,0];  % stay

  % Transitions(s,a,s')
  T = zeros(n^2,size(A,1),n^2);

  % loop over rows
  for i=1:n
    %i
    % loop over columns
    for j=1:n
      %j
      % loop over actions
      for i_a=1:size(A,1)
        %i_a
        i_s = sub2ind([n,n],i,j);
        trans = transition([i,j],A(i_a,:),n);

        %pause

        for t=1:size(trans,1)
          %t
          i_sp = sub2ind([n,n],trans(t,1),trans(t,2));
          T(i_s,i_a,i_sp) = trans(t,3);
        end

        %pause
      end
    end
  end

  % Rewards(s,a,s')
  R = zeros(n^2,size(A,1),n^2);

  % loop over rows
  for i=1:n
    % loop over columns
    for j=1:n
      % loop over actions
      for i_a=1:size(A,1)
        i_s = sub2ind([n,n],i,j);

        sp = [i,j] + A(i_a,:);
        if sp(1) >= 1 && sp(1) <= n && sp(2) >= 1 && sp(2) <= n
          i_sp = sub2ind([n,n],sp(1),sp(2));
          R(i_s,i_a,i_sp) = 1;
        end

        %pause
      end
    end
  end

  R(:,:,3) = -1000*R(:,:,3);
  R(:,:,4) = -1000*R(:,:,4);
  R(:,:,5) = -1000*R(:,:,5);
  R(:,:,6) = -1000*R(:,:,6);
  
  R(:,:,9) = -1000*R(:,:,9);
  R(:,:,10) = -1000*R(:,:,10);
  R(:,:,11) = -1000*R(:,:,11);
  R(:,:,12) = -1000*R(:,:,12);
  
  R(:,:,15) = -1000*R(:,:,15);
  R(:,:,16) = -1000*R(:,:,16);
  R(:,:,17) = -1000*R(:,:,17);
  R(:,:,18) = -1000*R(:,:,18);
  
  R(:,:,21) = -1000*R(:,:,21);
  R(:,:,22) = -1000*R(:,:,22);
  R(:,:,23) = -1000*R(:,:,23);
  R(:,:,24) = -1000*R(:,:,24);
  
  R(:,:,n^2) = 100*R(:,:,n^2);

  % Observations(s',o')
  Z = diag(ones(n^2,1)) + ones(n^2);
  Z = normr(Z);

  % return values
  sol.A = A;
  sol.T = T;
  sol.R = R;
  sol.Z = Z;
return