 clear all;
    close all;
    format long g;
    clc;
		
    
    
% % 	IMU_FindRQ('GraDes_nor', 'EKF_4-gyro', 18); 	
% % 	IMU_FindRQ('TilCom', 'EKF_4-gyro', 20); 	
% % 	IMU_FindRQ('GraDes_adv', 'EKF_4-gyro', 18);
% 
% % 	IMU_FindRQ('TilCom',     'EKF_7-bias', 15);
% 	IMU_FindRQ('GraDes_nor', 'EKF_7-bias', 10.6, 5.6, 13.4); % good
% 	IMU_FindRQ('GraDes_adv', 'EKF_7-bias', 5.8, 5.8, 13.4); 
%     
%     
% % 	IMU_FindRQ('TilCom', 'EKF_7-gyro', 15);
% 	IMU_FindRQ('GraDes_nor', 'EKF_7-gyro', 1.7, 1.3, 10.5); % good
% 	IMU_FindRQ('GraDes_adv', 'EKF_7-gyro',  2, 2, 11); <-



	IMU_FindRQ(2, 'TilCom',     'EKF_7-bias', 6.6, 6.6, 20.4);
	IMU_FindRQ(2, 'GraDes_nor', 'EKF_7-bias', 10.6, 5.6, 13.4); % good
	IMU_FindRQ(2, 'GraDes_adv', 'EKF_7-bias', 5.8, 5.8, 13.4); 