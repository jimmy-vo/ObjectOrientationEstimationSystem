function [RMSE] = Data_Plot_RMS(isplot, NAME, Result, Compare, NumberofSample)
	
	inore = 90;
	Error = zeros(NumberofSample, 3);
	
	for sample=1:NumberofSample     
		Error(sample,:) =  [wrapTo180(Result(sample,1) - Compare(sample,1)) , wrapTo180(Result(sample,2) - Compare(sample,2)) , wrapTo180(Result(sample,3) - Compare(sample,3)) ];
	end 

	%RMS
	
	RMSE = zeros(1, 3);
    for sample = 1 : NumberofSample
		if abs (Error(sample,1)) > inore 
			Error(sample,1) = 0;
		end
		if abs (Error(sample,2)) > inore
			Error(sample,2) = 0;
		end
		if abs (Error(sample,3)) > inore
			Error(sample,3) = 0;
		end
		RMSE(1) = RMSE(1) + Error(sample,1) * Error(sample,1); 
		RMSE(2) = RMSE(2) + Error(sample,2) * Error(sample,2);
		RMSE(3) = RMSE(3) + Error(sample,3) * Error(sample,3);
	end	
	RMSE = sqrt (RMSE/sample);
	
	
	
	%plot
    if (isplot==1)
        figure('name',NAME,'NumberTitle','off');
        subplot(2,3,1);
        plot(1:NumberofSample, Compare(1:NumberofSample,1), 'r-', 1:NumberofSample, Result(1:NumberofSample,1), 'b-')
        title('Roll');

        subplot(2,3,2);
        plot(1:NumberofSample, Compare(1:NumberofSample,2), 'r-', 1:NumberofSample, Result(1:NumberofSample,2), 'b-');
        title('Pitch');

        subplot(2,3,3);
        plot(1:NumberofSample, Compare(1:NumberofSample,3), 'r-', 1:NumberofSample, Result(1:NumberofSample,3), 'b-');
        title('Yaw');

        subplot(2,3,4);
        plot(1:NumberofSample, Error(1:NumberofSample,1));
        title('Roll Error');

        subplot(2,3,5);
        plot(1:NumberofSample, Error(1:NumberofSample,2));
        title('Pitch Error');

        subplot(2,3,6);
        plot(1:NumberofSample, Error(1:NumberofSample,3));
        title('Yaw Error');
    end
    
	disp(' ');
     disp(strcat(NAME,' RMS:'));
	disp(RMSE);
end 

