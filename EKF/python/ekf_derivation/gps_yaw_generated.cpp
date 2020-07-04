// Sub Expressions
float S0 = sinf(ant_yaw);
float S1 = q0*q3;
float S2 = q1*q2;
float S3 = 2*S0*(S1 - S2);
float S4 = cosf(ant_yaw);
float S5 = powf(q1, 2);
float S6 = powf(q2, 2);
float S7 = powf(q0, 2) - powf(q3, 2);
float S8 = S4*(S5 - S6 + S7);
float S9 = S3 - S8;
float S10 = 1.0F/S9;
float S11 = S4*q0;
float S12 = S0*q3;
float S13 = S0*(-S5 + S6 + S7) + 2*S4*(S1 + S2);
float S14 = S10*S13;
float S15 = S0*q0 + S4*q3;
float S16 = S10*(S14*(S11 - S12) + S15);
float S17 = powf(S13, 2)/powf(S9, 2) + 1;
float S18 = 2/S17;
float S19 = 1.0F/(-S3 + S8);
float S20 = S4*q1;
float S21 = S0*q2;
float S22 = S13*S19;
float S23 = S0*q1 - S4*q2;
float S24 = S19*(S22*(S20 + S21) + S23);
float S25 = S19*(-S20 - S21 + S22*S23);
float S26 = S10*(-S11 + S12 + S14*S15);
float S27 = -P(0,0)*S16 - P(0,1)*S24 - P(0,2)*S25 + P(0,3)*S26;
float S28 = -P(0,1)*S16 - P(1,1)*S24 - P(1,2)*S25 + P(1,3)*S26;
float S29 = 4/powf(S17, 2);
float S30 = -P(0,2)*S16 - P(1,2)*S24 - P(2,2)*S25 + P(2,3)*S26;
float S31 = -P(0,3)*S16 - P(1,3)*S24 - P(2,3)*S25 + P(3,3)*S26;
float S32 = S18/(R_YAW - S16*S27*S29 - S24*S28*S29 - S25*S29*S30 + S26*S29*S31);


// Observation Jacobians
H_YAW(0) = -S16*S18;
H_YAW(1) = -S18*S24;
H_YAW(2) = -S18*S25;
H_YAW(3) = S18*S26;
H_YAW(4) = 0;
H_YAW(5) = 0;
H_YAW(6) = 0;
H_YAW(7) = 0;
H_YAW(8) = 0;
H_YAW(9) = 0;
H_YAW(10) = 0;
H_YAW(11) = 0;
H_YAW(12) = 0;
H_YAW(13) = 0;
H_YAW(14) = 0;
H_YAW(15) = 0;
H_YAW(16) = 0;
H_YAW(17) = 0;
H_YAW(18) = 0;
H_YAW(19) = 0;
H_YAW(20) = 0;
H_YAW(21) = 0;
H_YAW(22) = 0;
H_YAW(23) = 0;


// Kalman gains
Kfusion(0) = S27*S32;
Kfusion(1) = S28*S32;
Kfusion(2) = S30*S32;
Kfusion(3) = S31*S32;
Kfusion(4) = S32*(-P(0,4)*S16 - P(1,4)*S24 - P(2,4)*S25 + P(3,4)*S26);
Kfusion(5) = S32*(-P(0,5)*S16 - P(1,5)*S24 - P(2,5)*S25 + P(3,5)*S26);
Kfusion(6) = S32*(-P(0,6)*S16 - P(1,6)*S24 - P(2,6)*S25 + P(3,6)*S26);
Kfusion(7) = S32*(-P(0,7)*S16 - P(1,7)*S24 - P(2,7)*S25 + P(3,7)*S26);
Kfusion(8) = S32*(-P(0,8)*S16 - P(1,8)*S24 - P(2,8)*S25 + P(3,8)*S26);
Kfusion(9) = S32*(-P(0,9)*S16 - P(1,9)*S24 - P(2,9)*S25 + P(3,9)*S26);
Kfusion(10) = S32*(-P(0,10)*S16 - P(1,10)*S24 - P(2,10)*S25 + P(3,10)*S26);
Kfusion(11) = S32*(-P(0,11)*S16 - P(1,11)*S24 - P(2,11)*S25 + P(3,11)*S26);
Kfusion(12) = S32*(-P(0,12)*S16 - P(1,12)*S24 - P(2,12)*S25 + P(3,12)*S26);
Kfusion(13) = S32*(-P(0,13)*S16 - P(1,13)*S24 - P(2,13)*S25 + P(3,13)*S26);
Kfusion(14) = S32*(-P(0,14)*S16 - P(1,14)*S24 - P(2,14)*S25 + P(3,14)*S26);
Kfusion(15) = S32*(-P(0,15)*S16 - P(1,15)*S24 - P(2,15)*S25 + P(3,15)*S26);
Kfusion(16) = S32*(-P(0,16)*S16 - P(1,16)*S24 - P(2,16)*S25 + P(3,16)*S26);
Kfusion(17) = S32*(-P(0,17)*S16 - P(1,17)*S24 - P(2,17)*S25 + P(3,17)*S26);
Kfusion(18) = S32*(-P(0,18)*S16 - P(1,18)*S24 - P(2,18)*S25 + P(3,18)*S26);
Kfusion(19) = S32*(-P(0,19)*S16 - P(1,19)*S24 - P(2,19)*S25 + P(3,19)*S26);
Kfusion(20) = S32*(-P(0,20)*S16 - P(1,20)*S24 - P(2,20)*S25 + P(3,20)*S26);
Kfusion(21) = S32*(-P(0,21)*S16 - P(1,21)*S24 - P(2,21)*S25 + P(3,21)*S26);
Kfusion(22) = S32*(-P(0,22)*S16 - P(1,22)*S24 - P(2,22)*S25 + P(3,22)*S26);
Kfusion(23) = S32*(-P(0,23)*S16 - P(1,23)*S24 - P(2,23)*S25 + P(3,23)*S26);


