function [q] = Convert_DcmToQuaternion(dcm)

m11 = dcm(1); 	m12 = dcm(2); 	m13 = dcm(3); 	
m21 = dcm(4); 	m22 = dcm(5); 	m23 = dcm(6); 	
m31 = dcm(7); 	m32 = dcm(8); 	m33 = dcm(8); 

% m00 = dcm(1); 	m01 = -dcm(2); 	m02 = -dcm(3); 	
% m10 = -dcm(4); 	m11 = dcm(5); 	m12 = dcm(6); 	
% m20 = -dcm(7); 	m21 = dcm(8); 	m22 = dcm(8); 

if (1+m11+m22+m33 > 0)  
  S = sqrt(1+m11+m22+m33)/2; 
  q0 = S;
  q1 = (m32 - m23) / (4*S);
  q2 = (m13 - m31) / (4*S); 
  q3 = (m21 - m12) / (4*S); 
elseif (1+m11-m22-m33>0)  
  S = sqrt(1+m11-m22-m33)/2;  
  q0 = (m32 - m23) / (4*S);
  q1 = S;
  q2 = (m12 + m21) / (4*S); 
  q3 = (m13 + m31) / (4*S); 
elseif (1-m11+m22-m33)  
  S = sqrt(1-m11+m22-m33)/2; 
  q0 = (m13 - m31) / (4*S);
  q1 = (m12 + m21) / (4*S); 
  q2 = S;
  q3 = (m23 + m32) / (4*S); 
else  
  S = sqrt(1-m11-m22+m33)/2;  
  q0 = (m21 - m12) / (4*S);
  q1 = (m13 + m31) / (4*S); 
  q2 = (m23 + m32) / (4*S); 
  q3 = S;
end
    

     q = [q0, -q1, -q2, -q3];

end

