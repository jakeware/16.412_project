function testdominate()
  n = 3;
  m = 10;
  G = rand(n,m);
  pi = randi([1 m]);
  p = G(:,pi);
  bargmax = dominate(p,G);
  if ~bargmax
    display('the vector is dominated')
    return;
  end
  assert(sum(bargmax)-1<1e-5);
  display('optimal belief is valid')
  
%   G
%   i
%   G'*bargmax'
  
  [~,maxi] = max(G'*bargmax');
  assert((maxi==pi));
  display('optimal action is correct')
end

