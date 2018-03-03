function [Roll Pitch Yaw] = Convert_QuaternionToEuler(q)

	RAD_TO_DEG = 57.2957795;
    
    q0=q(1);
    q1=q(2);
    q2=q(3);
    q3=q(4);
    
	Roll  = atan2 (2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2)) * RAD_TO_DEG ;
    Pitch = asin   ( min(max(real( (-2*(q1*q3 - q0*q2)) ),-1),1)) * RAD_TO_DEG ;
    Yaw   = atan2 (2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2)) * RAD_TO_DEG ;
    
end

