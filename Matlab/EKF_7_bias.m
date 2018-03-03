function [Out_Qua X_minus P_minus] = EKF_7_bias(EKF_dt, R, Q, EKF_Qua, Pre_Qua, Raw_Gyro, X_minus, P_minus, sample)
		%update linearized matrix & transition discrete transition matrix		
        timecst = 2;
        q0 = EKF_Qua(sample-1,1);
        q1 = EKF_Qua(sample-1,2);
        q2 = EKF_Qua(sample-1,3);
        q3 = EKF_Qua(sample-1,4);
        x = Raw_Gyro(sample,1) - X_minus(5);
        y = Raw_Gyro(sample,2) - X_minus(6);
        z = Raw_Gyro(sample,3) - X_minus(7);
        
        F = ... 	
        [ ...
              1,            -x*EKF_dt/2,    -y*EKF_dt/2,    -z*EKF_dt/2,    q1*EKF_dt/2,    q2*EKF_dt/2,    q3*EKF_dt/2 ; ...	
              x*EKF_dt/2    1,              z*EKF_dt/2,     -y*EKF_dt/2,    -q0*EKF_dt/2,   q3*EKF_dt/2,    -q2*EKF_dt/2; ...	
              y*EKF_dt/2,   -z*EKF_dt/2,    1,              x*EKF_dt/2,     -q3*EKF_dt/2,   -q0*EKF_dt/2,    q1*EKF_dt/2; ...	
              z*EKF_dt/2,   y*EKF_dt/2,     -x*EKF_dt/2     1,              q2*EKF_dt/2,    -q1*EKF_dt/2,   -q0*EKF_dt/2 ; ...	
              0,            0,              0,              0,              exp(-EKF_dt/timecst),              0,              0            ;...  
              0,            0,              0,              0,              0,              exp(-EKF_dt/timecst),              0            ;...  
              0,            0,              0,              0,              0,              0,              exp(-EKF_dt/timecst)            ;...                   
        ];  

        %update Kalman Gain
        K = P_minus* inv(P_minus + R);

        %%update Posteriori state    
        z_measurement   = [Pre_Qua(sample,:) zeros(1,3)]';    
        z_estimate      = [X_minus(1:4)' zeros(1,3)]';   
        X_plus = X_minus + K * (z_measurement - z_estimate);
        X_plus = X_plus/norm(X_plus);
        
        %update Posteriori Covariance
        P_plus = (eye(7,7) - K)*P_minus;
        
        %Extract result 
        Out_Qua = [X_plus(1), X_plus(2), X_plus(3), X_plus(4)]/norm(X_plus(1:4));
        
        %Predict Priori Covariance
        P_minus = F *	P_plus * F' + Q;    

        %Predict Priori state
        X_minus = F * X_plus;     		


end 

