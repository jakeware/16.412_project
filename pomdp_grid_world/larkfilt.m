function [U] = larkfilt(G)
  % prunes a set of alpha vectors and their
  % corresponding action sequences using Lark's
  % filtering algorithm
  
  n = size(G,1);
  U = zeros(n,0);
  unchecked = G;
  while size(unchecked,2)>0
    pi = randi([1 size(unchecked,2)]);
    p = unchecked(:,pi);
    x = dominate(p,U);
    if (~x)
      unchecked(:,pi) = [];
    else
      [pstar,pstari] = best(x,unchecked);
      U = [U,pstar];
      unchecked(:,pstari) = [];
    end
  end
end

function [vargmax,vargmaxi] = best(b,P)
  [~,vargmaxi] = max(P'*b');
  vargmax = P(:,vargmaxi);
end
