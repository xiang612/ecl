// Sub Expressions - axis 0
float HK0 = -magD*q2 + magE*q3 + magN*q0;
float HK1 = magD*q3 + magE*q2 + magN*q1;
float HK2 = magE*q1;
float HK3 = magD*q0;
float HK4 = magN*q2;
float HK5 = magD*q1 + magE*q0 - magN*q3;
float HK6 = powf(q0, 2) + powf(q1, 2) - powf(q2, 2) - powf(q3, 2);
float HK7 = q0*q3 + q1*q2;
float HK8 = q1*q3;
float HK9 = q0*q2;
float HK10 = 2*HK7;
float HK11 = -2*HK8 + 2*HK9;
float HK12 = 2*HK1;
float HK13 = 2*HK0;
float HK14 = -2*HK2 + 2*HK3 + 2*HK4;
float HK15 = 2*HK5;
float HK16 = HK10*P(0,17) - HK11*P(0,18) + HK12*P(0,1) + HK13*P(0,0) - HK14*P(0,2) + HK15*P(0,3) + HK6*P(0,16) + P(0,19);
float HK17 = HK10*P(16,17) - HK11*P(16,18) + HK12*P(1,16) + HK13*P(0,16) - HK14*P(2,16) + HK15*P(3,16) + HK6*P(16,16) + P(16,19);
float HK18 = HK10*P(17,18) - HK11*P(18,18) + HK12*P(1,18) + HK13*P(0,18) - HK14*P(2,18) + HK15*P(3,18) + HK6*P(16,18) + P(18,19);
float HK19 = HK10*P(2,17) - HK11*P(2,18) + HK12*P(1,2) + HK13*P(0,2) - HK14*P(2,2) + HK15*P(2,3) + HK6*P(2,16) + P(2,19);
float HK20 = HK10*P(17,17) - HK11*P(17,18) + HK12*P(1,17) + HK13*P(0,17) - HK14*P(2,17) + HK15*P(3,17) + HK6*P(16,17) + P(17,19);
float HK21 = HK10*P(3,17) - HK11*P(3,18) + HK12*P(1,3) + HK13*P(0,3) - HK14*P(2,3) + HK15*P(3,3) + HK6*P(3,16) + P(3,19);
float HK22 = HK10*P(1,17) - HK11*P(1,18) + HK12*P(1,1) + HK13*P(0,1) - HK14*P(1,2) + HK15*P(1,3) + HK6*P(1,16) + P(1,19);
float HK23 = HK10*P(17,19) - HK11*P(18,19) + HK12*P(1,19) + HK13*P(0,19) - HK14*P(2,19) + HK15*P(3,19) + HK6*P(16,19) + P(19,19);
float HK24 = 1.0F/(HK10*HK20 - HK11*HK18 + HK12*HK22 + HK13*HK16 - HK14*HK19 + HK15*HK21 + HK17*HK6 + HK23 + R_MAG);


// Observation Jacobians - axis 0
H_TAS(0) = 2*HK0;
H_TAS(1) = 2*HK1;
H_TAS(2) = 2*HK2 - 2*HK3 - 2*HK4;
H_TAS(3) = 2*HK5;
H_TAS(4) = 0;
H_TAS(5) = 0;
H_TAS(6) = 0;
H_TAS(7) = 0;
H_TAS(8) = 0;
H_TAS(9) = 0;
H_TAS(10) = 0;
H_TAS(11) = 0;
H_TAS(12) = 0;
H_TAS(13) = 0;
H_TAS(14) = 0;
H_TAS(15) = 0;
H_TAS(16) = HK6;
H_TAS(17) = 2*HK7;
H_TAS(18) = 2*HK8 - 2*HK9;
H_TAS(19) = 1;
H_TAS(20) = 0;
H_TAS(21) = 0;
H_TAS(22) = 0;
H_TAS(23) = 0;


// Kalman gains - axis 0
Kfusion(0) = HK16*HK24;
Kfusion(1) = HK22*HK24;
Kfusion(2) = HK19*HK24;
Kfusion(3) = HK21*HK24;
Kfusion(4) = HK24*(HK10*P(4,17) - HK11*P(4,18) + HK12*P(1,4) + HK13*P(0,4) - HK14*P(2,4) + HK15*P(3,4) + HK6*P(4,16) + P(4,19));
Kfusion(5) = HK24*(HK10*P(5,17) - HK11*P(5,18) + HK12*P(1,5) + HK13*P(0,5) - HK14*P(2,5) + HK15*P(3,5) + HK6*P(5,16) + P(5,19));
Kfusion(6) = HK24*(HK10*P(6,17) - HK11*P(6,18) + HK12*P(1,6) + HK13*P(0,6) - HK14*P(2,6) + HK15*P(3,6) + HK6*P(6,16) + P(6,19));
Kfusion(7) = HK24*(HK10*P(7,17) - HK11*P(7,18) + HK12*P(1,7) + HK13*P(0,7) - HK14*P(2,7) + HK15*P(3,7) + HK6*P(7,16) + P(7,19));
Kfusion(8) = HK24*(HK10*P(8,17) - HK11*P(8,18) + HK12*P(1,8) + HK13*P(0,8) - HK14*P(2,8) + HK15*P(3,8) + HK6*P(8,16) + P(8,19));
Kfusion(9) = HK24*(HK10*P(9,17) - HK11*P(9,18) + HK12*P(1,9) + HK13*P(0,9) - HK14*P(2,9) + HK15*P(3,9) + HK6*P(9,16) + P(9,19));
Kfusion(10) = HK24*(HK10*P(10,17) - HK11*P(10,18) + HK12*P(1,10) + HK13*P(0,10) - HK14*P(2,10) + HK15*P(3,10) + HK6*P(10,16) + P(10,19));
Kfusion(11) = HK24*(HK10*P(11,17) - HK11*P(11,18) + HK12*P(1,11) + HK13*P(0,11) - HK14*P(2,11) + HK15*P(3,11) + HK6*P(11,16) + P(11,19));
Kfusion(12) = HK24*(HK10*P(12,17) - HK11*P(12,18) + HK12*P(1,12) + HK13*P(0,12) - HK14*P(2,12) + HK15*P(3,12) + HK6*P(12,16) + P(12,19));
Kfusion(13) = HK24*(HK10*P(13,17) - HK11*P(13,18) + HK12*P(1,13) + HK13*P(0,13) - HK14*P(2,13) + HK15*P(3,13) + HK6*P(13,16) + P(13,19));
Kfusion(14) = HK24*(HK10*P(14,17) - HK11*P(14,18) + HK12*P(1,14) + HK13*P(0,14) - HK14*P(2,14) + HK15*P(3,14) + HK6*P(14,16) + P(14,19));
Kfusion(15) = HK24*(HK10*P(15,17) - HK11*P(15,18) + HK12*P(1,15) + HK13*P(0,15) - HK14*P(2,15) + HK15*P(3,15) + HK6*P(15,16) + P(15,19));
Kfusion(16) = HK17*HK24;
Kfusion(17) = HK20*HK24;
Kfusion(18) = HK18*HK24;
Kfusion(19) = HK23*HK24;
Kfusion(20) = HK24*(HK10*P(17,20) - HK11*P(18,20) + HK12*P(1,20) + HK13*P(0,20) - HK14*P(2,20) + HK15*P(3,20) + HK6*P(16,20) + P(19,20));
Kfusion(21) = HK24*(HK10*P(17,21) - HK11*P(18,21) + HK12*P(1,21) + HK13*P(0,21) - HK14*P(2,21) + HK15*P(3,21) + HK6*P(16,21) + P(19,21));
Kfusion(22) = HK24*(HK10*P(17,22) - HK11*P(18,22) + HK12*P(1,22) + HK13*P(0,22) - HK14*P(2,22) + HK15*P(3,22) + HK6*P(16,22) + P(19,22));
Kfusion(23) = HK24*(HK10*P(17,23) - HK11*P(18,23) + HK12*P(1,23) + HK13*P(0,23) - HK14*P(2,23) + HK15*P(3,23) + HK6*P(16,23) + P(19,23));


// Sub Expressions - axis 1
float HK0 = -magD*q2 + magE*q3 + magN*q0;
float HK1 = magD*q3 + magE*q2 + magN*q1;
float HK2 = magE*q1;
float HK3 = magD*q0;
float HK4 = magN*q2;
float HK5 = magD*q1 + magE*q0 - magN*q3;
float HK6 = powf(q0, 2) + powf(q1, 2) - powf(q2, 2) - powf(q3, 2);
float HK7 = q0*q3 + q1*q2;
float HK8 = q1*q3;
float HK9 = q0*q2;
float HK10 = 2*HK7;
float HK11 = -2*HK8 + 2*HK9;
float HK12 = 2*HK1;
float HK13 = 2*HK0;
float HK14 = -2*HK2 + 2*HK3 + 2*HK4;
float HK15 = 2*HK5;
float HK16 = HK10*P(0,17) - HK11*P(0,18) + HK12*P(0,1) + HK13*P(0,0) - HK14*P(0,2) + HK15*P(0,3) + HK6*P(0,16) + P(0,19);
float HK17 = HK10*P(16,17) - HK11*P(16,18) + HK12*P(1,16) + HK13*P(0,16) - HK14*P(2,16) + HK15*P(3,16) + HK6*P(16,16) + P(16,19);
float HK18 = HK10*P(17,18) - HK11*P(18,18) + HK12*P(1,18) + HK13*P(0,18) - HK14*P(2,18) + HK15*P(3,18) + HK6*P(16,18) + P(18,19);
float HK19 = HK10*P(2,17) - HK11*P(2,18) + HK12*P(1,2) + HK13*P(0,2) - HK14*P(2,2) + HK15*P(2,3) + HK6*P(2,16) + P(2,19);
float HK20 = HK10*P(17,17) - HK11*P(17,18) + HK12*P(1,17) + HK13*P(0,17) - HK14*P(2,17) + HK15*P(3,17) + HK6*P(16,17) + P(17,19);
float HK21 = HK10*P(3,17) - HK11*P(3,18) + HK12*P(1,3) + HK13*P(0,3) - HK14*P(2,3) + HK15*P(3,3) + HK6*P(3,16) + P(3,19);
float HK22 = HK10*P(1,17) - HK11*P(1,18) + HK12*P(1,1) + HK13*P(0,1) - HK14*P(1,2) + HK15*P(1,3) + HK6*P(1,16) + P(1,19);
float HK23 = HK10*P(17,19) - HK11*P(18,19) + HK12*P(1,19) + HK13*P(0,19) - HK14*P(2,19) + HK15*P(3,19) + HK6*P(16,19) + P(19,19);
float HK24 = 1.0F/(HK10*HK20 - HK11*HK18 + HK12*HK22 + HK13*HK16 - HK14*HK19 + HK15*HK21 + HK17*HK6 + HK23 + R_MAG);


// Observation Jacobians - axis 1
H_TAS(0) = 2*HK0;
H_TAS(1) = 2*HK1;
H_TAS(2) = 2*HK2 - 2*HK3 - 2*HK4;
H_TAS(3) = 2*HK5;
H_TAS(4) = 0;
H_TAS(5) = 0;
H_TAS(6) = 0;
H_TAS(7) = 0;
H_TAS(8) = 0;
H_TAS(9) = 0;
H_TAS(10) = 0;
H_TAS(11) = 0;
H_TAS(12) = 0;
H_TAS(13) = 0;
H_TAS(14) = 0;
H_TAS(15) = 0;
H_TAS(16) = HK6;
H_TAS(17) = 2*HK7;
H_TAS(18) = 2*HK8 - 2*HK9;
H_TAS(19) = 1;
H_TAS(20) = 0;
H_TAS(21) = 0;
H_TAS(22) = 0;
H_TAS(23) = 0;


// Kalman gains - axis 1
Kfusion(0) = HK16*HK24;
Kfusion(1) = HK22*HK24;
Kfusion(2) = HK19*HK24;
Kfusion(3) = HK21*HK24;
Kfusion(4) = HK24*(HK10*P(4,17) - HK11*P(4,18) + HK12*P(1,4) + HK13*P(0,4) - HK14*P(2,4) + HK15*P(3,4) + HK6*P(4,16) + P(4,19));
Kfusion(5) = HK24*(HK10*P(5,17) - HK11*P(5,18) + HK12*P(1,5) + HK13*P(0,5) - HK14*P(2,5) + HK15*P(3,5) + HK6*P(5,16) + P(5,19));
Kfusion(6) = HK24*(HK10*P(6,17) - HK11*P(6,18) + HK12*P(1,6) + HK13*P(0,6) - HK14*P(2,6) + HK15*P(3,6) + HK6*P(6,16) + P(6,19));
Kfusion(7) = HK24*(HK10*P(7,17) - HK11*P(7,18) + HK12*P(1,7) + HK13*P(0,7) - HK14*P(2,7) + HK15*P(3,7) + HK6*P(7,16) + P(7,19));
Kfusion(8) = HK24*(HK10*P(8,17) - HK11*P(8,18) + HK12*P(1,8) + HK13*P(0,8) - HK14*P(2,8) + HK15*P(3,8) + HK6*P(8,16) + P(8,19));
Kfusion(9) = HK24*(HK10*P(9,17) - HK11*P(9,18) + HK12*P(1,9) + HK13*P(0,9) - HK14*P(2,9) + HK15*P(3,9) + HK6*P(9,16) + P(9,19));
Kfusion(10) = HK24*(HK10*P(10,17) - HK11*P(10,18) + HK12*P(1,10) + HK13*P(0,10) - HK14*P(2,10) + HK15*P(3,10) + HK6*P(10,16) + P(10,19));
Kfusion(11) = HK24*(HK10*P(11,17) - HK11*P(11,18) + HK12*P(1,11) + HK13*P(0,11) - HK14*P(2,11) + HK15*P(3,11) + HK6*P(11,16) + P(11,19));
Kfusion(12) = HK24*(HK10*P(12,17) - HK11*P(12,18) + HK12*P(1,12) + HK13*P(0,12) - HK14*P(2,12) + HK15*P(3,12) + HK6*P(12,16) + P(12,19));
Kfusion(13) = HK24*(HK10*P(13,17) - HK11*P(13,18) + HK12*P(1,13) + HK13*P(0,13) - HK14*P(2,13) + HK15*P(3,13) + HK6*P(13,16) + P(13,19));
Kfusion(14) = HK24*(HK10*P(14,17) - HK11*P(14,18) + HK12*P(1,14) + HK13*P(0,14) - HK14*P(2,14) + HK15*P(3,14) + HK6*P(14,16) + P(14,19));
Kfusion(15) = HK24*(HK10*P(15,17) - HK11*P(15,18) + HK12*P(1,15) + HK13*P(0,15) - HK14*P(2,15) + HK15*P(3,15) + HK6*P(15,16) + P(15,19));
Kfusion(16) = HK17*HK24;
Kfusion(17) = HK20*HK24;
Kfusion(18) = HK18*HK24;
Kfusion(19) = HK23*HK24;
Kfusion(20) = HK24*(HK10*P(17,20) - HK11*P(18,20) + HK12*P(1,20) + HK13*P(0,20) - HK14*P(2,20) + HK15*P(3,20) + HK6*P(16,20) + P(19,20));
Kfusion(21) = HK24*(HK10*P(17,21) - HK11*P(18,21) + HK12*P(1,21) + HK13*P(0,21) - HK14*P(2,21) + HK15*P(3,21) + HK6*P(16,21) + P(19,21));
Kfusion(22) = HK24*(HK10*P(17,22) - HK11*P(18,22) + HK12*P(1,22) + HK13*P(0,22) - HK14*P(2,22) + HK15*P(3,22) + HK6*P(16,22) + P(19,22));
Kfusion(23) = HK24*(HK10*P(17,23) - HK11*P(18,23) + HK12*P(1,23) + HK13*P(0,23) - HK14*P(2,23) + HK15*P(3,23) + HK6*P(16,23) + P(19,23));


// Sub Expressions - axis 2
float HK0 = -magD*q2 + magE*q3 + magN*q0;
float HK1 = magD*q3 + magE*q2 + magN*q1;
float HK2 = magE*q1;
float HK3 = magD*q0;
float HK4 = magN*q2;
float HK5 = magD*q1 + magE*q0 - magN*q3;
float HK6 = powf(q0, 2) + powf(q1, 2) - powf(q2, 2) - powf(q3, 2);
float HK7 = q0*q3 + q1*q2;
float HK8 = q1*q3;
float HK9 = q0*q2;
float HK10 = 2*HK7;
float HK11 = -2*HK8 + 2*HK9;
float HK12 = 2*HK1;
float HK13 = 2*HK0;
float HK14 = -2*HK2 + 2*HK3 + 2*HK4;
float HK15 = 2*HK5;
float HK16 = HK10*P(0,17) - HK11*P(0,18) + HK12*P(0,1) + HK13*P(0,0) - HK14*P(0,2) + HK15*P(0,3) + HK6*P(0,16) + P(0,19);
float HK17 = HK10*P(16,17) - HK11*P(16,18) + HK12*P(1,16) + HK13*P(0,16) - HK14*P(2,16) + HK15*P(3,16) + HK6*P(16,16) + P(16,19);
float HK18 = HK10*P(17,18) - HK11*P(18,18) + HK12*P(1,18) + HK13*P(0,18) - HK14*P(2,18) + HK15*P(3,18) + HK6*P(16,18) + P(18,19);
float HK19 = HK10*P(2,17) - HK11*P(2,18) + HK12*P(1,2) + HK13*P(0,2) - HK14*P(2,2) + HK15*P(2,3) + HK6*P(2,16) + P(2,19);
float HK20 = HK10*P(17,17) - HK11*P(17,18) + HK12*P(1,17) + HK13*P(0,17) - HK14*P(2,17) + HK15*P(3,17) + HK6*P(16,17) + P(17,19);
float HK21 = HK10*P(3,17) - HK11*P(3,18) + HK12*P(1,3) + HK13*P(0,3) - HK14*P(2,3) + HK15*P(3,3) + HK6*P(3,16) + P(3,19);
float HK22 = HK10*P(1,17) - HK11*P(1,18) + HK12*P(1,1) + HK13*P(0,1) - HK14*P(1,2) + HK15*P(1,3) + HK6*P(1,16) + P(1,19);
float HK23 = HK10*P(17,19) - HK11*P(18,19) + HK12*P(1,19) + HK13*P(0,19) - HK14*P(2,19) + HK15*P(3,19) + HK6*P(16,19) + P(19,19);
float HK24 = 1.0F/(HK10*HK20 - HK11*HK18 + HK12*HK22 + HK13*HK16 - HK14*HK19 + HK15*HK21 + HK17*HK6 + HK23 + R_MAG);


// Observation Jacobians - axis 2
H_TAS(0) = 2*HK0;
H_TAS(1) = 2*HK1;
H_TAS(2) = 2*HK2 - 2*HK3 - 2*HK4;
H_TAS(3) = 2*HK5;
H_TAS(4) = 0;
H_TAS(5) = 0;
H_TAS(6) = 0;
H_TAS(7) = 0;
H_TAS(8) = 0;
H_TAS(9) = 0;
H_TAS(10) = 0;
H_TAS(11) = 0;
H_TAS(12) = 0;
H_TAS(13) = 0;
H_TAS(14) = 0;
H_TAS(15) = 0;
H_TAS(16) = HK6;
H_TAS(17) = 2*HK7;
H_TAS(18) = 2*HK8 - 2*HK9;
H_TAS(19) = 1;
H_TAS(20) = 0;
H_TAS(21) = 0;
H_TAS(22) = 0;
H_TAS(23) = 0;


// Kalman gains - axis 2
Kfusion(0) = HK16*HK24;
Kfusion(1) = HK22*HK24;
Kfusion(2) = HK19*HK24;
Kfusion(3) = HK21*HK24;
Kfusion(4) = HK24*(HK10*P(4,17) - HK11*P(4,18) + HK12*P(1,4) + HK13*P(0,4) - HK14*P(2,4) + HK15*P(3,4) + HK6*P(4,16) + P(4,19));
Kfusion(5) = HK24*(HK10*P(5,17) - HK11*P(5,18) + HK12*P(1,5) + HK13*P(0,5) - HK14*P(2,5) + HK15*P(3,5) + HK6*P(5,16) + P(5,19));
Kfusion(6) = HK24*(HK10*P(6,17) - HK11*P(6,18) + HK12*P(1,6) + HK13*P(0,6) - HK14*P(2,6) + HK15*P(3,6) + HK6*P(6,16) + P(6,19));
Kfusion(7) = HK24*(HK10*P(7,17) - HK11*P(7,18) + HK12*P(1,7) + HK13*P(0,7) - HK14*P(2,7) + HK15*P(3,7) + HK6*P(7,16) + P(7,19));
Kfusion(8) = HK24*(HK10*P(8,17) - HK11*P(8,18) + HK12*P(1,8) + HK13*P(0,8) - HK14*P(2,8) + HK15*P(3,8) + HK6*P(8,16) + P(8,19));
Kfusion(9) = HK24*(HK10*P(9,17) - HK11*P(9,18) + HK12*P(1,9) + HK13*P(0,9) - HK14*P(2,9) + HK15*P(3,9) + HK6*P(9,16) + P(9,19));
Kfusion(10) = HK24*(HK10*P(10,17) - HK11*P(10,18) + HK12*P(1,10) + HK13*P(0,10) - HK14*P(2,10) + HK15*P(3,10) + HK6*P(10,16) + P(10,19));
Kfusion(11) = HK24*(HK10*P(11,17) - HK11*P(11,18) + HK12*P(1,11) + HK13*P(0,11) - HK14*P(2,11) + HK15*P(3,11) + HK6*P(11,16) + P(11,19));
Kfusion(12) = HK24*(HK10*P(12,17) - HK11*P(12,18) + HK12*P(1,12) + HK13*P(0,12) - HK14*P(2,12) + HK15*P(3,12) + HK6*P(12,16) + P(12,19));
Kfusion(13) = HK24*(HK10*P(13,17) - HK11*P(13,18) + HK12*P(1,13) + HK13*P(0,13) - HK14*P(2,13) + HK15*P(3,13) + HK6*P(13,16) + P(13,19));
Kfusion(14) = HK24*(HK10*P(14,17) - HK11*P(14,18) + HK12*P(1,14) + HK13*P(0,14) - HK14*P(2,14) + HK15*P(3,14) + HK6*P(14,16) + P(14,19));
Kfusion(15) = HK24*(HK10*P(15,17) - HK11*P(15,18) + HK12*P(1,15) + HK13*P(0,15) - HK14*P(2,15) + HK15*P(3,15) + HK6*P(15,16) + P(15,19));
Kfusion(16) = HK17*HK24;
Kfusion(17) = HK20*HK24;
Kfusion(18) = HK18*HK24;
Kfusion(19) = HK23*HK24;
Kfusion(20) = HK24*(HK10*P(17,20) - HK11*P(18,20) + HK12*P(1,20) + HK13*P(0,20) - HK14*P(2,20) + HK15*P(3,20) + HK6*P(16,20) + P(19,20));
Kfusion(21) = HK24*(HK10*P(17,21) - HK11*P(18,21) + HK12*P(1,21) + HK13*P(0,21) - HK14*P(2,21) + HK15*P(3,21) + HK6*P(16,21) + P(19,21));
Kfusion(22) = HK24*(HK10*P(17,22) - HK11*P(18,22) + HK12*P(1,22) + HK13*P(0,22) - HK14*P(2,22) + HK15*P(3,22) + HK6*P(16,22) + P(19,22));
Kfusion(23) = HK24*(HK10*P(17,23) - HK11*P(18,23) + HK12*P(1,23) + HK13*P(0,23) - HK14*P(2,23) + HK15*P(3,23) + HK6*P(16,23) + P(19,23));


