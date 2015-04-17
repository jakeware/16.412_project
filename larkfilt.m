function [U,K] = larkfilt(G,A)
  % prunes a set of alpha vectors and their
  % corresponding action sequences using Lark's
  % filtering algorithm
  
  n = size(G,1);
  U = zeros(n,0);
  K = zeros(size(A,1),0);
  unchecked = G;
  uncheckedi = 1:size(G,2);
  while size(unchecked,2)>0
    pi = randi([1 size(unchecked,2)]);
    p = unchecked(:,pi);
    x = dominate(p,U);
    if (~x)
      unchecked(:,pi) = [];
      uncheckedi(:,pi) = [];
    else
      [pstar,pstari] = best(x,unchecked);
      U = [U,pstar];
      K = [K,A(:,uncheckedi(:,pstari))];
      unchecked(:,pstari) = [];
      uncheckedi(:,pstari) = [];
    end
  end
end

function [vargmax,vargmaxi] = best(b,P)
  [~,vargmaxi] = max(P'*b');
  vargmax = P(:,vargmaxi);
end
