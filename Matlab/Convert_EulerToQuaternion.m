function [Quaternion] = Convert_EulerToQuaternion(Roll, Pitch, Yaw)

    DEG_TO_RAD = 0.0174532925;

    Roll  = Roll    *DEG_TO_RAD/2;
    Pitch = Pitch   *DEG_TO_RAD/2;
    Yaw   = Yaw     *DEG_TO_RAD/2;
	
	Q0 = cos(Roll)*cos(Pitch)*cos(Yaw) + sin(Roll)*sin(Pitch)*sin(Yaw);
	Q1 = sin(Roll)*cos(Pitch)*cos(Yaw) - cos(Roll)*sin(Pitch)*sin(Yaw);
	Q2 = cos(Roll)*sin(Pitch)*cos(Yaw) + sin(Roll)*cos(Pitch)*sin(Yaw);
	Q3 = cos(Roll)*cos(Pitch)*sin(Yaw) - sin(Roll)*sin(Pitch)*cos(Yaw);
	
	Quaternion = [Q0;Q1;Q2;Q3];
end

