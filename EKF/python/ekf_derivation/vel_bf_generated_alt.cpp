// Sub Expressions
float S0 = q0*vn;
float S1 = q3*ve;
float S2 = q2*vd;
float S3 = 2*S0 + 2*S1 - 2*S2;
float S4 = 2*(q1*vn + q2*ve + q3*vd);
float S5 = q1*ve;
float S6 = q0*vd;
float S7 = q2*vn;
float S8 = q0*ve;
float S9 = q1*vd;
float S10 = q3*vn;
float S11 = -2*S10 + 2*S8 + 2*S9;
float S12 = powf(q1, 2);
float S13 = powf(q2, 2);
float S14 = -S13;
float S15 = powf(q0, 2);
float S16 = powf(q3, 2);
float S17 = S15 - S16;
float S18 = q0*q3;
float S19 = q1*q2;
float S20 = q1*q3;
float S21 = q0*q2;
float S22 = -2*S5 + 2*S6 + 2*S7;
float S23 = -S12;
float S24 = q0*q1;
float S25 = q2*q3;


// X axis observation Jacobians and Kalman gains
H_VEL(0) = S3;
H_VEL(1) = S4;
H_VEL(2) = 2*S5 - 2*S6 - 2*S7;
H_VEL(3) = S11;
H_VEL(4) = S12 + S14 + S17;
H_VEL(5) = 2*S18 + 2*S19;
H_VEL(6) = 2*S20 - 2*S21;
H_VEL(7) = 0;
H_VEL(8) = 0;
H_VEL(9) = 0;
H_VEL(10) = 0;
H_VEL(11) = 0;
H_VEL(12) = 0;
H_VEL(13) = 0;
H_VEL(14) = 0;
H_VEL(15) = 0;
H_VEL(16) = 0;
H_VEL(17) = 0;
H_VEL(18) = 0;
H_VEL(19) = 0;
H_VEL(20) = 0;
H_VEL(21) = 0;
H_VEL(22) = 0;
H_VEL(23) = 0;


Kfusion(0) = 0;
Kfusion(1) = 0;
Kfusion(2) = 0;
Kfusion(3) = 0;
Kfusion(4) = 0;
Kfusion(5) = 0;
Kfusion(6) = 0;
Kfusion(7) = 0;
Kfusion(8) = 0;
Kfusion(9) = 0;
Kfusion(10) = 0;
Kfusion(11) = 0;
Kfusion(12) = 0;
Kfusion(13) = 0;
Kfusion(14) = 0;
Kfusion(15) = 0;
Kfusion(16) = 0;
Kfusion(17) = 0;
Kfusion(18) = 0;
Kfusion(19) = 0;
Kfusion(20) = 0;
Kfusion(21) = 0;
Kfusion(22) = 0;
Kfusion(23) = 0;


// Y axis observation Jacobians and Kalman gains
H_VEL(0) = S11;
H_VEL(1) = S22;
H_VEL(2) = S4;
H_VEL(3) = -2*S0 - 2*S1 + 2*S2;
H_VEL(4) = -2*S18 + 2*S19;
H_VEL(5) = S13 + S17 + S23;
H_VEL(6) = 2*S24 + 2*S25;
H_VEL(7) = 0;
H_VEL(8) = 0;
H_VEL(9) = 0;
H_VEL(10) = 0;
H_VEL(11) = 0;
H_VEL(12) = 0;
H_VEL(13) = 0;
H_VEL(14) = 0;
H_VEL(15) = 0;
H_VEL(16) = 0;
H_VEL(17) = 0;
H_VEL(18) = 0;
H_VEL(19) = 0;
H_VEL(20) = 0;
H_VEL(21) = 0;
H_VEL(22) = 0;
H_VEL(23) = 0;


Kfusion(0) = 0;
Kfusion(1) = 0;
Kfusion(2) = 0;
Kfusion(3) = 0;
Kfusion(4) = 0;
Kfusion(5) = 0;
Kfusion(6) = 0;
Kfusion(7) = 0;
Kfusion(8) = 0;
Kfusion(9) = 0;
Kfusion(10) = 0;
Kfusion(11) = 0;
Kfusion(12) = 0;
Kfusion(13) = 0;
Kfusion(14) = 0;
Kfusion(15) = 0;
Kfusion(16) = 0;
Kfusion(17) = 0;
Kfusion(18) = 0;
Kfusion(19) = 0;
Kfusion(20) = 0;
Kfusion(21) = 0;
Kfusion(22) = 0;
Kfusion(23) = 0;


// Z axis observation Jacobians and Kalman gains
H_VEL(0) = S22;
H_VEL(1) = 2*S10 - 2*S8 - 2*S9;
H_VEL(2) = S3;
H_VEL(3) = S4;
H_VEL(4) = 2*S20 + 2*S21;
H_VEL(5) = -2*S24 + 2*S25;
H_VEL(6) = S14 + S15 + S16 + S23;
H_VEL(7) = 0;
H_VEL(8) = 0;
H_VEL(9) = 0;
H_VEL(10) = 0;
H_VEL(11) = 0;
H_VEL(12) = 0;
H_VEL(13) = 0;
H_VEL(14) = 0;
H_VEL(15) = 0;
H_VEL(16) = 0;
H_VEL(17) = 0;
H_VEL(18) = 0;
H_VEL(19) = 0;
H_VEL(20) = 0;
H_VEL(21) = 0;
H_VEL(22) = 0;
H_VEL(23) = 0;


Kfusion(0) = 0;
Kfusion(1) = 0;
Kfusion(2) = 0;
Kfusion(3) = 0;
Kfusion(4) = 0;
Kfusion(5) = 0;
Kfusion(6) = 0;
Kfusion(7) = 0;
Kfusion(8) = 0;
Kfusion(9) = 0;
Kfusion(10) = 0;
Kfusion(11) = 0;
Kfusion(12) = 0;
Kfusion(13) = 0;
Kfusion(14) = 0;
Kfusion(15) = 0;
Kfusion(16) = 0;
Kfusion(17) = 0;
Kfusion(18) = 0;
Kfusion(19) = 0;
Kfusion(20) = 0;
Kfusion(21) = 0;
Kfusion(22) = 0;
Kfusion(23) = 0;


