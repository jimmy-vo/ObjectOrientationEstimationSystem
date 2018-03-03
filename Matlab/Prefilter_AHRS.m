function [Quaternion, eInt] = Prefilter_AHRS(q, Gyro, Acce, Magn, eInt, Ki, Kp, SamplePeriod)
% 	Gyro = Gyro / 0.0174532925;

    % Normalise accelerometer measurement
	Accelerometer = Acce' / norm(Acce);    % normalise magnitude

	% Normalise magnetometer measurement
	Magnetometer = Magn' / norm(Magn);   % normalise magnitude

	% Reference direction of Earth's magnetic feild
	h = QuaternionProduct(q, QuaternionProduct([0 Magnetometer'], [q(1), -q(2), -q(3), -q(4)]));
	b = [0 norm([h(2) h(3)]) 0 h(4)];
	
	% Estimated direction of gravity and magnetic field
	v = [2*(q(2)*q(4) - q(1)*q(3))
		 2*(q(1)*q(2) + q(3)*q(4))
		 q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];
	w = [2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3))
		 2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4))
		 2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2)]; 

	% Error is sum of cross product between estimated direction and measured direction of fields
	e = cross(Accelerometer, v)' + cross(Magnetometer, w)'; 
	if(Ki > 0)
		eInt = eInt + e * SamplePeriod;   
	else
		eInt = [0 0 0];
	end
	
	% Apply feedback terms
	Gyroscope = Gyro + Kp * e + Ki * eInt;      
	
	% Compute rate of change of quaternion
	qDot = 0.5 * QuaternionProduct(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);

	% Integrate to yield quaternion
	Q = q + qDot' * SamplePeriod;
	Quaternion = Q / norm(Q); % normalise quaternion
end