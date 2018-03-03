function [Source Compare NumberofSample Raw_Acce Raw_Gyro Raw_Magn dt] = Data_Import(file)

	if (file == 1)
	source  = 'D:\_Clouds_\Dropbox\Thesis\Matlab\_DATA\raw\still.log';
	result = 'D:\_Clouds_\Dropbox\Thesis\Matlab\_DATA\result\still.log';
	disp('Test Condition: still');
	end
	if (file == 2)
	source  = 'D:\_Clouds_\Dropbox\Thesis\Matlab\_DATA\raw\free2.log';
	result = 'D:\_Clouds_\Dropbox\Thesis\Matlab\_DATA\result\free2.log';
	disp('Test Condition: free move');
	end
	if (file == 3)
	source  = 'D:\_Clouds_\Dropbox\Thesis\Matlab\_DATA\raw\free1.log';
	result = 'D:\_Clouds_\Dropbox\Thesis\Matlab\_DATA\result\free1.log';
	disp('Test Condition: free move in magnetic field');
	end
	if (file == 4)
	source  = 'D:\_Clouds_\Dropbox\Thesis\Matlab\_DATA\raw\xaxismove.log';
	result = 'D:\_Clouds_\Dropbox\Thesis\Matlab\_DATA\result\xaxismove.log';
	disp('Test Condition: moving along x axis');
	end
	if (file == 5)
	source  = 'D:\_Clouds_\Dropbox\Thesis\Matlab\_DATA\raw\zaxisturn.log';
	result = 'D:\_Clouds_\Dropbox\Thesis\Matlab\_DATA\result\zaxisturn.log';
	disp('Test Condition: moving along z axis');
	end 
    disp('============================================================================');

	fid = fopen(source);
	allText = textscan(fid,'%s','delimiter','\n');
	numberOfLines = length(allText{1});
	fclose(fid);

	Source = dlmread(source);
	ResultF = dlmread(result);
	Compare (:,1:3) = ResultF(:,2:4);

	%%define number of sample
	NumberofSample = numberOfLines;

	%% Sampling
	Raw_Acce = zeros(NumberofSample, 3);
	Raw_Gyro = zeros(NumberofSample, 3);
	Raw_Magn = zeros(NumberofSample, 3);
	
	%init dt
	dt = Source(3,1) - Source(2,1);

	for sample=1:NumberofSample    
		Raw_Acce(sample,:) = [Source(sample,2);Source(sample,3);Source(sample,4)];
		Raw_Gyro(sample,:) = [Source(sample,5);Source(sample,6);Source(sample,7)];
		Raw_Magn(sample,:) = [Source(sample,8);Source(sample,9);Source(sample,10)];
    end
    
    
% % normalize 
%     for sample=1:NumberofSample    
%         Raw_Magn(sample,:) = norm (Raw_Magn(sample));
%         Raw_Acce(sample,:) = norm (Raw_Acce(sample));
%     end
    
end 

