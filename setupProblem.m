function sol = setupProblem(n)
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

  % Tp(1,:,:) = [1,0,0,0;
  %             0,0,1,0;
  %             0,1,0,0;
  %             1,0,0,0;
  %             1,0,0,0];
  %           
  % Tp(2,:,:) = [1,0,0,0;
  %             0,0,0,1;
  %             0,1,0,0;
  %             0,1,0,0;
  %             0,1,0,0];
  %           
  % Tp(3,:,:) = [0,0,1,0;
  %             0,0,1,0;
  %             0,0,0,1;
  %             1,0,0,0;
  %             0,0,1,0];
  %           
  % Tp(4,:,:) = [0,0,1,0;
  %             0,0,0,1;
  %             0,0,0,1;
  %             0,1,0,0;
  %             0,0,0,1]; 

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
        trans = Transition([i,j],A(i_a,:),n);

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

  % Rp(1,:,:) = [0,0,0,0;
  %             0,0,1,0;
  %             0,1,0,0;
  %             0,0,0,0;
  %             1,0,0,0];
  % 
  % Rp(2,:,:) = [1,0,0,0;
  %             0,0,0,1;
  %             0,0,0,0;
  %             0,0,0,0;
  %             0,1,0,0];
  % 
  % Rp(3,:,:) = [0,0,0,0;
  %             0,0,0,0;
  %             0,0,0,1;
  %             1,0,0,0;
  %             0,0,1,0];
  % 
  % Rp(4,:,:) = [0,0,1,0;
  %             0,0,0,0;
  %             0,0,0,0;
  %             0,1,0,0;
  %             0,0,0,1];

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

  %for i = 1:n
  %  ind = sub2ind([n,n],1,i);
  %  R(:,:,ind) = 2;
  %end
  R(:,:,n^2) = 10*R(:,:,n^2);
%   R(:,:,1) = 1*R(:,:,1);
%   R(:,:,2) = 1*R(:,:,2);
%   R(:,:,3) = 2*R(:,:,3);
%   R(:,:,4) = -100*R(:,:,4);
%   R(:,:,5) = -100*R(:,:,5);
%   R(:,:,6) = 4*R(:,:,6);
%   R(:,:,7) = -100*R(:,:,7);
%   R(:,:,8) = -100*R(:,:,8);
%   R(:,:,9) = -100*R(:,:,9);
%   R(:,:,10) = 8*R(:,:,10);
%   R(:,:,11) = 6*R(:,:,11);
%   R(:,:,12) = -100*R(:,:,12);
%   R(:,:,13) = -100*R(:,:,13);
%   R(:,:,14) = 10*R(:,:,14);
%   R(:,:,15) = -100*R(:,:,15);
%   R(:,:,16) = -100*R(:,:,16);

  % Observations(s',o')
  Z = diag(ones(n^2,1)) + ones(n^2);
  Z = normr(Z);

  sol.A = A;
  sol.T = T;
  sol.R = R;
  sol.Z = Z;
return