function [Roll, Pitch, Yaw] = Prefilter_TiltCompensation(Acce, typea, Magn, inverse)
	RAD_TO_DEG = 57.2957795;
	
	accX = Acce(1,1);
	accY = Acce(2,1);
	accZ = Acce(3,1);	
	
%     %norm
% 	Magn(:) = norm (Magn);
    
	magX = Magn(1,1);
	magY = Magn(2,1);
	magZ = Magn(3,1);	
	
	%Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf 
	
	if typea == 1 % eq. 25 and eq. 26
	  Roll = atan2(accY, accZ) ;
	  Pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) ;
	else 		 % Eq. 28 and 29
	  Roll = atan(accY / sqrt(accX * accX + accZ * accZ)) ;
	  Pitch = atan2(-accX, accZ) ;
    end
    
	
    %Source: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
	xH = magX * cos(Pitch) + magY * sin(Roll) * sin(Pitch) + magZ * sin(Pitch) * cos(Roll);
	yH = magY * cos(Roll) - magZ * sin(Roll) ;
	Yaw = atan2(-yH,xH) ;
        
    Roll    = Roll * RAD_TO_DEG;
    Pitch   = Pitch * RAD_TO_DEG;
    Yaw     = Yaw * RAD_TO_DEG * inverse;
end

