function [RMSE_Pre, RMSE_EKF] = IMU_Algo_function(isplot, file, Pre_Filter, EKF_Filter, r1, r2, q1, q2)
% 		clear all;
% 		close all;
		format long g;


	%% init constant     
		
		if (strcmp(EKF_Filter, 'EKF_7-bias'))
			r = [ r1 * ones(1,4), r2 * ones(1,3)];
			q = [ q1 * ones(1,4), q2 * ones(1,3)];
		elseif (strcmp(EKF_Filter, 'EKF_7-gyro'))       
			r = [ r1 * ones(1,3), r2 * ones(1,4)];
			q = [ q1 * ones(1,3), q2 * ones(1,4)];
		elseif (strcmp(EKF_Filter, 'EKF_4-gyro'))
			r = [ r1 * ones(1,4), 0 * ones(1,3)];
			q = [ q1 * ones(1,4), 0 * ones(1,3)];
		end
		
	%% Init EKF
		
		% import 
		[source Compare NumberofSample Raw_Acce Raw_Gyro Raw_Magn EKF_dt] = DataImport(file);
		Pre_Qua = zeros(NumberofSample, 4);
		EKF_Qua = zeros(NumberofSample, 4);
		
		%init Measurement noise variance matrix
		Q7 = [ ...
				q(1),   0,      0,      0,      0,      0,      0   ; ...
				0,      q(2),   0,      0,      0,      0,      0   ; ...
				0,      0,      q(3),   0,      0,      0,      0   ; ...
				0,      0,      0,      q(4),   0,      0,      0   ; ...
				0,      0,      0,      0,      q(5),   0,      0   ; ...
				0,      0,      0,      0,      0,      q(6),   0   ; ...
				0,      0,      0,      0,      0,      0,      q(7); ...
			];
		Q4 = [ ...
				q(1),   0,		0,		0    ; ...
				0,      q(2),   0,		0    ; ...
				0,		0,		q(3),   0    ; ...
				0,		0,		0,		q(4) ; ...
			 ];

		%init process noise variance matrix
		R7 = [ ...
				r(1),   0,      0,		0,		0,		0,		0    ; ...
				0,      r(2),   0,      0,		0,		0,		0    ; ...
				0,      0,		r(3),   0,		0,		0,		0    ; ...
				0,      0,		0,		r(4),   0,		0,		0    ; ...
				0,      0,		0,		0,		r(5),   0,		0    ; ...
				0,      0,		0,		0,		0,		r(6),   0    ; ...
				0,  	0,		0,		0,		0,		0,		r(7) ; ...
			];
		R4 = [ ...
				r(1),   0,		0,		0    ; ...
				0,      r(2),   0,		0    ; ...
				0,		0,		r(3),   0    ; ...
				0,		0,		0,		r(4) ; ...
			];
		
		Pre_Qua(1,:)        = [1; 0; 0; 0];
		EKF_Qua(1,:)        = [1; 0; 0; 0];
		
		P_minus7            =   0.5 * eye(7,7);
		P_minus4            =   0.001 * eye(4,4);
		
		X_minus7            = [0; 0; 0; EKF_Qua(1,:)'];    
		X_minus4            = EKF_Qua(1,:)';
		
		AHRS_error          = zeros (1,3);
		
	%% Gradient Descent - EKF 7-bias
		for sample=2:NumberofSample    
			%Pre-filter 
			if      (strcmp(Pre_Filter, 'Ideal'))
				Pre_Qua(sample,:) = EulerToQuaternion (Compare(sample,1), Compare(sample,2), Compare(sample,3));   
			
			elseif  (strcmp(Pre_Filter, 'TilCom'))  
				[Raw_Roll, Raw_Pitch, Raw_Yaw] = TiltCompensation(Raw_Acce(sample,:)', 1, Raw_Magn(sample,:)', 0);    
				Pre_Qua(sample,:) = EulerToQuaternion (Raw_Roll, Raw_Pitch, Raw_Yaw );  
		   
			elseif  (strcmp(Pre_Filter, 'AHRS_nor'))   
				[Pre_Qua(sample,:),AHRS_error]  = AHRS (Pre_Qua(sample-1,:), Raw_Gyro(sample,:), Raw_Acce(sample,:), Raw_Magn(sample,:), AHRS_error, 0, 5, EKF_dt);

			elseif  (strcmp(Pre_Filter, 'AHRS_adv'))    
				[Pre_Qua(sample,:),AHRS_error]  = AHRS (EKF_Qua(sample-1,:), Raw_Gyro(sample,:), Raw_Acce(sample,:), Raw_Magn(sample,:), AHRS_error, 0, 5, EKF_dt);
			   
			elseif  (strcmp(Pre_Filter, 'GraDes_nor'))    
				dq = 0.5 * QuaternionProduct(Pre_Qua(sample-1,:),[0, Raw_Gyro(sample,:)]);
				mu = 10 * norm(dq) * EKF_dt;
				Pre_Qua(sample,:) = GradientDescent(Raw_Acce(sample,:), Raw_Magn(sample,:), Pre_Qua(sample-1,:), mu);   
			
			elseif  (strcmp(Pre_Filter, 'GraDes_adv'))        
				dq = 0.5 * QuaternionProduct(EKF_Qua(sample-1,:),[0, Raw_Gyro(sample,:)]);
				mu = 10 * norm(dq) * EKF_dt;
				Pre_Qua(sample,:) = GradientDescent(Raw_Acce(sample,:), Raw_Magn(sample,:), EKF_Qua(sample-1,:), mu);   
			end

			%EKF
			if      (strcmp(EKF_Filter, 'EKF_7-bias'))
				[EKF_Qua(sample,:) X_minus7 P_minus7] = EKF_7_bias (EKF_dt, R7, Q7, EKF_Qua, Pre_Qua, Raw_Gyro, X_minus7, P_minus7, sample);
			
			elseif  (strcmp(EKF_Filter, 'EKF_7-gyro'))
				[EKF_Qua(sample,:) X_minus7 P_minus7] = EKF_7_gyro (EKF_dt, R7, Q7, EKF_Qua, Pre_Qua, Raw_Gyro, X_minus7, P_minus7, sample);
			
			elseif  (strcmp(EKF_Filter, 'EKF_4-gyro'))
				[EKF_Qua(sample,:) X_minus4 P_minus4] = EKF_4_gyro (EKF_dt, R4, Q4, EKF_Qua, Pre_Qua, Raw_Gyro, X_minus4, P_minus4, sample);
			end
		end

	%% Convert to Euler

	Pre_Euler = zeros(NumberofSample, 3);    
	EKF_Euler = zeros(NumberofSample, 3);    

	for sample=1:NumberofSample     
		[roll, pitch, yaw] = QuaternionToEuler(Pre_Qua(sample,:));    
		Pre_Euler(sample,:)= [roll, pitch, yaw];
		[roll, pitch, yaw] = QuaternionToEuler(EKF_Qua(sample,:));  
		EKF_Euler(sample,:)= [roll, pitch, yaw];
	end 

	%% Console - Ploting
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		disp ([r;q]');
		
	RMSE_Pre = EulerPlot_RMS_VALUE(isplot, Pre_Filter, Pre_Euler, Compare, NumberofSample, 1000);
	RMSE_EKF = EulerPlot_RMS_VALUE(isplot, EKF_Filter, EKF_Euler, Compare, NumberofSample, 1000);

	%%
end