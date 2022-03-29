function [validIdx] = neighbourCheck(query,reference,delta)

% query - Mx2
% reference - Nx2
% validIdx - Mx1 Logical

mdl = KDTreeSearcher(reference);
[~,dist] = knnsearch(mdl, query,'K',1);
validIdx = dist > delta; 

end
