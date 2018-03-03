    clear all;
    close all;
    format long g;
    clc;
    
    file = 2   ;    
        
    Pre_Filter  = 'Ideal';     
    Pre_Filter  = 'Tilt_Compensate';     
     Pre_Filter  = 'Gauss_Newton';
%     Pre_Filter  = 'AHRS';  


    EKF_Filter  = 'EKF_4-gyro';
    EKF_Filter  = 'EKF_7-bias';
%     EKF_Filter  = 'EKF_7-gyro';

%% init constant     
    
    if (strcmp(EKF_Filter, 'EKF_7-bias'))   %%% OK
        r = [ 1.0e-6 * ones(1,4), 1e-04 * ones(1,3)];
        q = [ 0 * ones(1,4), 1e-9 * ones(1,3)];
    elseif (strcmp(EKF_Filter, 'EKF_7-gyro')) %% OK      
        r = [ 1.0e-4 * ones(1,3), 1e-06 * ones(1,4)]; 
        q = [ 1e-02 * ones(1,3), 0 * ones(1,4)];  
    elseif (strcmp(EKF_Filter, 'EKF_4-gyro'))
        r =  1.0e-6 * ones(1,4) ;
        q =  1.0e-2 * ones(1,4);
    end
    
%% Init EKF
    
    % import 
    [source Compare NumberofSample Raw_Acce Raw_Gyro Raw_Magn EKF_dt] = Data_Import(file);
    
    Pre_Qua = zeros(NumberofSample, 4);
    EKF_Qua = zeros(NumberofSample, 4); 
    Pre_Euler = zeros(NumberofSample, 3);    
    EKF_Euler = zeros(NumberofSample, 3);  
    
    AHRS_error          = zeros (1,3);
    
    %init Measurement noise variance matrix
    if (numel(q)==7)        Q7 = diag (q);
    else                    Q4 = diag (q);
    end

    %init process noise variance matrix    
    if (numel(r)==7)        R7 = diag (r);
    else                    R4 = diag (r);
    end
    
    Pre_Qua(1,:)        =  Convert_EulerToQuaternion (Compare(1,1), Compare(1,2), Compare(1,3) );  
    EKF_Qua(1,:)        =  Pre_Qua(1,:);
    
    P_minus7            =   0.05 * eye(7,7);
    P_minus4            =   0.001 * eye(4,4);
    
    X_minus7            = [0; 0; 0; EKF_Qua(1,:)'];    
    X_minus4            = EKF_Qua(1,:)';
    
    
%% Gradient Descent - EKF 7-bias
    cycle = cputime;
    for sample=2:NumberofSample    
        %Pre-filter 
        if      (strcmp(Pre_Filter, 'Ideal'))
            Pre_Qua(sample,:) = EulerToQuaternion (Compare(sample,1), Compare(sample,2), Compare(sample,3));   
        
        elseif  (strcmp(Pre_Filter, 'Tilt_Compensate'))  
            [Raw_Roll, Raw_Pitch, Raw_Yaw] = Prefilter_TiltCompensation(Raw_Acce(sample,:)', 1, Raw_Magn(sample,:)', 1);    
            Pre_Qua(sample,:) = Convert_EulerToQuaternion (Raw_Roll, Raw_Pitch, Raw_Yaw );  
       
        elseif  (strcmp(Pre_Filter, 'AHRS'))   
            [Pre_Qua(sample,:),AHRS_error]  = Prefilter_AHRS (Pre_Qua(sample-1,:), Raw_Gyro(sample,:), Raw_Acce(sample,:), Raw_Magn(sample,:), AHRS_error, 0, 5, EKF_dt);

        elseif  (strcmp(Pre_Filter, 'Gauss_Newton'))    
            Pre_Qua(sample,:) = GaussNewtonMethod(Pre_Qua(sample-1,:)',Raw_Acce(sample,:)',Raw_Magn(sample,:)');
        
        end

        %EKF
        if  (strcmp(EKF_Filter, 'EKF_7-gyro'))
            [EKF_Qua(sample,:) X_minus7 P_minus7] = EKF_7_gyro (EKF_dt, R7, Q7, EKF_Qua, Pre_Qua, Raw_Gyro, X_minus7, P_minus7, sample);
        
        elseif      (strcmp(EKF_Filter, 'EKF_7-bias'))
            [EKF_Qua(sample,:) X_minus7 P_minus7] = EKF_7_bias (EKF_dt, R7, Q7, EKF_Qua, Pre_Qua, Raw_Gyro, X_minus7, P_minus7, sample);
        
        elseif  (strcmp(EKF_Filter, 'EKF_4-gyro'))
            [EKF_Qua(sample,:) X_minus4 P_minus4] = EKF_4_gyro (EKF_dt, R4, Q4, EKF_Qua, Pre_Qua, Raw_Gyro, X_minus4, P_minus4, sample);
        end
        
        % Convert to Euler   
        [roll, pitch, yaw] = Convert_QuaternionToEuler(Pre_Qua(sample,:));    
        Pre_Euler(sample,:)= [roll, pitch, yaw];
        [roll, pitch, yaw] = Convert_QuaternionToEuler(EKF_Qua(sample,:));  
        EKF_Euler(sample,:)= [roll, pitch, yaw];
    end
    cycle = (cputime - cycle)/NumberofSample;

%% Console - Ploting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    disp(' ');
       
%Display alg   
    disp(strcat('Diagram:       [',Pre_Filter,'] -> [', EKF_Filter,']'));
    disp(' ');
    disp('Cycle (us):');
    disp(cycle*1e6);
 
%Display R Q   
    disp('_____________________________________________________________________');  
    disp(strcat(EKF_Filter,' constant:     R                         Q'));   
    disp ([r;q]');
    
% Plot 
    disp('_____________________________________________________________________'); 
    RMSE_Pre = Data_Plot_RMS(1, Pre_Filter, Pre_Euler, Compare, NumberofSample);
    RMSE_EKF = Data_Plot_RMS(1, EKF_Filter, EKF_Euler, Compare, NumberofSample);

%%