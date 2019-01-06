function [state_uncertainty] = ...
    compute_state_uncertainty(propagated_states,propagated_measurements, ...
    apriori_state, apriori_measurement)
    removed_state_expectation = propagated_states-apriori_state;
    removed_measurement_expectation = propagated_measurements-apriori_measurement;
    state_uncertainty = (SigmaPointConstants.W_c_0 * ...
        (removed_state_expectation(:,1)))*(removed_measurement_expectation(:,1))' + ...
        (SigmaPointConstants.W_i * ...
        (removed_state_expectation(:, 2:end)))*(removed_measurement_expectation(:, 2:end))';
end

