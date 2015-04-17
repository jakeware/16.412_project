n = 4;
m = 16;
G = 10*rand(n,m);

tic
U = larkfilt(G);
toc

display('unfiltered set:')
display(G)
display('filtered set:')
display(U)