function [X_minus, P_minus,  X_plus] = EKF_DCM( X_minus, P_minus, Q, R, PHI, obs)

    % compute Kalman gain
    K = P_minus * inv(P_minus + R);

    %%update Posteriori state 
    X_plus = X_minus + K*(obs - X_minus);
    X_plus(:) = X_plus(:)/ norm(X_plus(:));

    %update Posteriori Covariance
    P_plus = (eye(3,3) - K)*P_minus;
    

    %Predict Priori state
    X_minus = PHI * X_plus;
%     X_minus(:) = X_minus(:)/ norm(X_minus(:));

    %Predict Priori Covariance
    P_minus  = PHI * P_plus * PHI' + Q;

end