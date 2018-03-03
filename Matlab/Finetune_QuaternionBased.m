function [out] = IMU_FindRQ (file, Pre_Filter, EKF_Filter, limit_roll, limit_pitch, limit_yaw)
    format long g; 

    link = 'C:\Users\VND\Desktop\File_2\';
    fid = fopen(strcat(link,EKF_Filter,'__',Pre_Filter,'.txt'),'wt');
% 
%         Pre_Filter  = 'Ideal';   
%         Pre_Filter  = 'TilCom';
%         Pre_Filter  = 'GraDes_nor';
%         Pre_Filter  = 'GraDes_adv'; 
%         Pre_Filter  = 'AHRS_adv'; 
%         Pre_Filter  = 'AHRS_nor';    
% 
%         EKF_Filter  = 'EKF_4-gyro';
%         EKF_Filter  = 'EKF_7-gyro';
%         EKF_Filter  = 'EKF_7-bias';

        MAX              = 1000;
        lowr             = 0.0000001;
        highr            = 0.0001;
        
        lowq             = 0.00000001;
        highq            = 0.1;
        
        step             = 10;

        R = zeros(MAX, 2);
        Q = zeros(MAX, 2);
        RMS = zeros(MAX, 3);

        index = 1;
        r1 = lowr;
        r2 = lowr;
        q1 = lowq;
        q2 = lowq;



            while r1<highr
                while r2<highr
                    while q1<highq
                        while q2<highq
                            if (index<MAX+1)
                                clc; disp(index);
                                [RMSE_Pre, RMSE_EKF] = IMU_Algo_function(0, file, Pre_Filter, EKF_Filter, r1, r2, q1, q2);
                                if ((RMSE_EKF(1)<limit_roll)&&(RMSE_EKF(2)<limit_pitch)&&(RMSE_EKF(3)<limit_yaw))                                     
                                    R(index,:)      = [r1 r2];
                                    Q(index,:)      = [q1 q2];                                
                                    RMS(index,:)    = RMSE_EKF;
                                    disp(' ');
                                    index = index+1;
                                end
                            end
                            q2 = q2*step;
                        end
                        q1 = q1*step;
                        q2 = lowq;
                    end
                    r2 = r2*step;
                    q1 = lowq;
                    q2 = lowq;
                end
                r1 = r1*step;  
                r2 = lowr;
                q1 = lowq;
                q2 = lowq;     
            end

     A = [RMS R Q];
     index = 1;

    fprintf(fid,'file: %d --- r: %g p: %g y: %g \n \n', file, limit_roll, limit_pitch, limit_yaw);
    
    for ii = 1:size(A,1)
        fprintf(fid,'%d \t %g \t',index, A(ii,:));
        fprintf(fid,'\n');
        index = index+1;
    end
    fclose(fid);
end

	%%