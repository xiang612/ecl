// Equations for NE velocity Kalman gain
const float S0 = powf(P01, 2);
const float S1 = P11 + velObsVar;
const float S2 = P00 + velObsVar;
const float S3 = 1.0F/(S0 - S1*S2);
const float S4 = -P01*S3*velObsVar;


K(0,0) = S3*(-P00*S1 + S0);
K(1,0) = S4;
K(2,0) = S3*(P01*P12 - P02*S1);
K(0,1) = S4;
K(1,1) = S3*(-P11*S2 + S0);
K(2,1) = S3*(P01*P02 - P12*S2);


