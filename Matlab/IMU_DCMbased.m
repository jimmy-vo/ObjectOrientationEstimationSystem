    clear all;
    close all;
    format long g;
    clc;
    
    file = 2;    
  
    
%% Init EKF
    
    % import 
    [source Compare NumberofSample Raw_Acce Raw_Gyro Raw_Magn EKF_dt] = Data_Import(file);
	   
	% measurement noise
	q   = 1e-5 * ones(1,3); 
	ra  = 1e-5 * ones(1,3); 
	rm  = 1e-5 * ones(1,3); 

	Racc = diag(ra);
	Rmag = diag(rm);

	% process noise
	Q = EKF_dt^2 * diag(q);
    
    %%%%%%%%%
    
    % initial parameters 
    
	EKF_Euler 	= zeros(NumberofSample, 3);    
    a = [Compare(1,1), Compare(1,2), Compare(1,3)]*pi/180; 

    p0 = 1e-2;
    Pi10  = diag([p0 p0 p0]);
    
    p0 = 1e-2;
    Pi20  = diag([p0 p0 p0]);
    
    xi10  = [-sin(a(1,2)); sin(a(1,1))*cos(a(1,2)); cos(a(1,1))*cos(a(1,2))];
    
    z1 = cos(a(2))*sin(a(3));
    z2 = sin(a(1))*sin(a(2))*sin(a(3)) + cos(a(2))*cos(a(3));
    z3 = cos(a(1))*sin(a(2))*sin(a(3)) - sin(a(1))*cos(a(3));
    xi20  = [z1; z2; z3];
    
	X1_minus 		= xi10;  
	P1_minus 		= Pi10; 
    
	X2_minus 		= xi20;  
	P2_minus 		= Pi20;  
	
    
%% Gradient Descent - EKF 7-bias
    cycle = cputime;
    for sample=2:NumberofSample    
        
        
        % update transition matrix 
            PHI = [ 
                    0                     Raw_Gyro(sample,3)      -Raw_Gyro(sample,2);
                   -Raw_Gyro(sample,3)    0                       Raw_Gyro(sample,1);
                    Raw_Gyro(sample,2)    -Raw_Gyro(sample,1)     0				
                  ];
            PHI = eye(3) + PHI*EKF_dt;
        
        % prepare obs1  
            obs = Raw_Acce(sample,:)/9.81;
%             obs = Raw_Acce(sample,:)/norm(Raw_Acce(sample,:));
        
        %EKF1
            [X1_minus P1_minus X1_plus] = EKF_DCM(X1_minus, P1_minus, Q, Racc, PHI, obs');

        % prepare obs2  
            phi = atan2(X1_plus(2),X1_plus(3));
            s_phi = sin(phi);
            c_phi = cos(phi);

            s_theta = -X1_plus(1);             
            c_theta = sqrt(1-s_theta^2);    

            Xh = Raw_Magn(sample,1)*c_theta + Raw_Magn(sample,2)*s_theta*s_phi + Raw_Magn(sample,3)*s_theta*c_phi;
            Yh = Raw_Magn(sample,2)*c_phi - Raw_Magn(sample,3)*s_phi;
            s_psi = -Yh / sqrt(Xh^2 + Yh^2);
            c_psi = Xh / sqrt(Xh^2 + Yh^2);

            obs(1) = c_theta*s_psi;
            obs(2) = s_phi*s_theta*s_psi + c_phi*c_psi;
            obs(3) = c_phi*s_theta*s_psi - s_phi*c_psi;
        
        %EKF2
            [X2_minus, P2_minus, X2_plus ] = EKF_DCM(X2_minus, P2_minus, Q, Rmag, PHI, obs');

            C11 = X2_plus(2)*X1_plus(3) - X2_plus(3)*X1_plus(2);
            C12 = X2_plus(3)*X1_plus(1) - X2_plus(1)*X1_plus(3);
            C13 = X2_plus(1)*X1_plus(2) - X2_plus(2)*X1_plus(1);

            C11 = C11 / sqrt(C11^2 + C12^2 + C13^2);
            C12 = C12 / sqrt(C11^2 + C12^2 + C13^2);
            C13 = C13 / sqrt(C11^2 + C12^2 + C13^2);

            dcm = [ 
                    C11,            C12,            C13; 
                    X2_plus(1),     X2_plus(2),     X2_plus(3); 
                    X1_plus(1),     X1_plus(2),     X1_plus(3)
                  ];

            EKF_Euler(sample,1) = atan2(X1_plus(2),X1_plus(3))   *180/pi;   
            EKF_Euler(sample,2) = -asin(X1_plus(1))              *180/pi;        
            EKF_Euler(sample,3) = atan2(X2_plus(1), C11)         *180/pi;
    end 
   cycle = (cputime - cycle)/NumberofSample;
   
PHI
PHI'
%% Console - Ploting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Display alg   
    disp('______________________________________________________________________________');
    disp('Diagram:       [Kalman filter 1] -> [Kalman filter 2]');
    disp(' ');
    disp('Cycle (us):');
    disp(cycle*1e6);
    
 
%Display R Q   
    disp('______________________________________________________________________________');  
    disp(' constant:            Racc                      Rmag                         Q');   
    disp ([ra;rm;q]');
%     
% Plot     
    disp('______________________________________________________________________________'); 
    RMSE_EKF = Data_Plot_RMS(1, 'DCM', EKF_Euler, Compare, NumberofSample);

%%