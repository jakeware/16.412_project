function bargmax = dominate(p,G)
  % uses an LP to test for dominance

  % p is the vector you are checking for dominance against G
  % dom is whether the vector dominates at any point in the belief space
  
  % decision variables b (belief state) and d (difference in value)
  % [s1 s2 s3 ... d]

  n = size(G,1); % number of states
  if size(G,2)<1
    bargmax = rand(1,n);
    return;
  end
  
  % remove p from G if there
  G = G(:,~all(ismember(G,p),1));
  m = size(G,2);
  
  % max d
  c = [zeros(n,1);-1];
  
  % sum constraint checking for dominance
  A = [repmat(p',m,1)-G',-1*ones(m,1)];
  blc = zeros(m,1);
  buc = Inf*ones(m,1);
  
  % belief sums up to one (valid prob distribution)
  A = [A;ones(1,n),0];
  blc = [blc;1];
  buc = [buc;1];
  
  % pobs are between 0-1
  blx = [zeros(n,1);-Inf];
  bux = [ones(n,1);Inf];
  
  % solve the LP
  clear prob;
  prob.c = c;
  prob.a = A;
  prob.blc = blc;
  prob.buc = buc;
  prob.blx = blx;
  prob.bux = bux;
  [~,res] = mosekopt('minimize echo(0)',prob);
  sol = res.sol;
  
  x = sol.itr.xx';
  d = x(end);
  if (d>0)
    bargmax = x(1:n);
  else
    bargmax = false;
  end
  
end

