function [X_minus, P_minus,  X_plus] = EKF_2( X_minus, P_minus, Q, R, obs)

    % compute Kalman gain
    K = P_minus * inv(P_minus + R);

    %%update Posteriori state 
    X_plus = X_minus + K*(obs - X_minus);
    X_plus(:) = X_plus(:)/ norm(X_plus(:));

    %update Posteriori Covariance
    P_plus = (eye(2,2) - K)*P_minus;

    %Predict Priori state
    X_minus = F * X_plus;

    %Predict Priori Covariance
    P_minus  = F * P_plus * PHI' + Q;
end