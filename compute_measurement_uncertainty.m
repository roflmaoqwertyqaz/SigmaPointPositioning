function [measurement_uncertainty] = compute_measurement_uncertainty(propagated_measurements,apriori_measurement)
    removed_expectation = propagated_measurements-apriori_measurement;
    measurement_uncertainty = (SigmaPointConstants.W_c_0 * ...
        (removed_expectation(:,1)))*(removed_expectation(:,1))' + ...
        (SigmaPointConstants.W_i * ...
        (removed_expectation(:, 2:end)))*(removed_expectation(:, 2:end))';
end

