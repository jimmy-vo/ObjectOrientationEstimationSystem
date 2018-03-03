function [Out_Qua X_minus P_minus] = EKF_7_gyro(EKF_dt, R, Q, EKF_Qua, Pre_Qua, Raw_Gyro, X_minus, P_minus, sample)
	%update linearized matrix & transition discrete transition matrix	
    
    
    timecst =  2 ;
    
	x1 = Raw_Gyro(sample,1);
	x2 = Raw_Gyro(sample,2);
	x3 = Raw_Gyro(sample,3);
	x4 = EKF_Qua(sample-1,1);
	x5 = EKF_Qua(sample-1,2);
	x6 = EKF_Qua(sample-1,3);
	x7 = EKF_Qua(sample-1,4);
    
	F = ... 	
    [ ... 
        exp(-EKF_dt/timecst)    ,0,						0,						0,						0,						0,						0           ; ...
        0, 						exp(-EKF_dt/timecst),   0,						0,						0,						0,						0           ; ...	
        0,						0,						exp(-EKF_dt/timecst),   0,						0,						0,						0           ; ...	
        -x5*EKF_dt/2,           -x6*EKF_dt/2,			-x7*EKF_dt/2,			1,						-x1*EKF_dt/2,			-x2*EKF_dt/2,			-x3*EKF_dt/2; ...	
         x4*EKF_dt/2,			-x7*EKF_dt/2,			 x6*EKF_dt/2,			 x1*EKF_dt/2,	   		1,                  	 x3*EKF_dt/2,			-x2*EKF_dt/2; ...	
         x7*EKF_dt/2,			 x4*EKF_dt/2,			-x5*EKF_dt/2,			 x2*EKF_dt/2,			-x3*EKF_dt/2,			1,                       x1*EKF_dt/2; ...	
        -x6*EKF_dt/2,			 x5*EKF_dt/2,			 x4*EKF_dt/2,			 x3*EKF_dt/2,			 x2*EKF_dt/2,			-x1*EKF_dt/2,			1           ; ...	
    ];     
        
    z_measurement   = [0; 0; 0; Pre_Qua(sample,:)'];    
    z_estimate      = [0; 0; 0; EKF_Qua(sample,:)'];
	
	%update Kalman Gain
    K = P_minus*inv(P_minus + R);
	
	%%update Posteriori state    
	X_plus = X_minus + K * (z_measurement - z_estimate);
%     X_plus = X_plus/norm(X_plus);
    	
	%update Posteriori Covariance
	P_plus = (eye(7,7) - K)*P_minus;
    
	%Extract result 
    Out_Qua = [X_plus(4), X_plus(5), X_plus(6), X_plus(7)]/norm(X_plus(4:7));
    
    %Predict Priori state
 	X_minus = F * X_plus;    
%     X_minus = X_minus/norm(X_minus); 		
		
	%Predict Priori Covariance
    P_minus = F *	P_plus * F' + Q;    
    
end 

