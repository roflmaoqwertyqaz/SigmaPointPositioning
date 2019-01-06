function [sigma_points] = generate_sigma_points(state,covariance)
    
    [u, s, v] = svd(covariance);
    cov_sqrt = SigmaPointConstants.gamma *  u * sqrt(s) ;
    sigma_points = [state, ...
               repmat(state, 1, SigmaPointConstants.L) + cov_sqrt, ...
               repmat(state, 1, SigmaPointConstants.L) - cov_sqrt];
end

