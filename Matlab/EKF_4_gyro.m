function [Out_Qua X_minus P_minus] = EKF_4_gyro(EKF_dt, R, Q, EKF_Qua, Pre_Qua, Raw_Gyro, X_minus, P_minus, sample)
    %update discrete transition matrix	    	      
    const = EKF_dt/2;
    
    F1 = [    1                           -const*Raw_Gyro(sample,1)     -const*Raw_Gyro(sample,2)     -const*Raw_Gyro(sample,3) ];
    F2 = [    const*Raw_Gyro(sample,1)    1                             const*Raw_Gyro(sample,3)      -const*Raw_Gyro(sample,2) ];
    F3 = [    const*Raw_Gyro(sample,2)    -const*Raw_Gyro(sample,3)     1                             const*Raw_Gyro(sample,1)  ];
    F4 = [    -const*Raw_Gyro(sample,3)   const*Raw_Gyro(sample,2)      -const*Raw_Gyro(sample,1)     1                         ];    
    
    
	F=[F1;F2;F3;F4];
    
	%update Kalman Gain
    K = P_minus*inv(P_minus + R);
	
	%%update Posteriori state    
	X_plus = X_minus + K * (Pre_Qua(sample,:)' - EKF_Qua(sample,:)');
%     X_plus = X_plus/norm(X_plus);
%     
	%update Posteriori Covariance
	P_plus = (eye(4,4) - K)*P_minus;
    
    %Extract result 
    Out_Qua = [X_plus(1), X_plus(2), X_plus(3), X_plus(4)]/norm(X_plus);
    
    %Predict Priori state
 	X_minus = F * X_plus;   
%     X_minus = X_minus/norm(X_minus);
    
	%Predict Priori Covariance
    P_minus = F *	P_plus * F' + Q;    
    
	
end 

