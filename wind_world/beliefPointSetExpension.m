function B = beliefPointSetExpension(b0,T,maxsize)
  
  m = size(T,2);
  B = b0;
  while size(B,2)<maxsize
    for bi=1:size(B,2)
      bset = [];
      bsetdist = [];
      for ai=1:m
        newb = squeeze(T(:,ai,:))'*B(:,bi);
        bset = [bset,newb];
        bsetdist = [bsetdist,max(sum((B-repmat(newb,1,size(B,2))).^2))];
      end
      [~,maxbi] = max(bsetdist);
      B = [B,bset(:,maxbi)];
    end
  end
  
end

