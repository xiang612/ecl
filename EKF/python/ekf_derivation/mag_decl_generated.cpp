// Sub Expressions
float S0 = powf(magN, -2);
float S1 = S0*powf(magE, 2) + 1;
float S2 = 1.0F/S1;
float S3 = 1.0F/magN;
float S4 = S2*S3;
float S5 = S3*magE;
float S6 = P(16,17)*S5 - P(17,17);
float S7 = powf(S1, -2);
float S8 = P(16,16)*S5 - P(16,17);
float S9 = S4/(R_DECL - S0*S6*S7 + S7*S8*magE/powf(magN, 3));


// Observation Jacobians
H_DECL(0) = 0;
H_DECL(1) = 0;
H_DECL(2) = 0;
H_DECL(3) = 0;
H_DECL(4) = 0;
H_DECL(5) = 0;
H_DECL(6) = 0;
H_DECL(7) = 0;
H_DECL(8) = 0;
H_DECL(9) = 0;
H_DECL(10) = 0;
H_DECL(11) = 0;
H_DECL(12) = 0;
H_DECL(13) = 0;
H_DECL(14) = 0;
H_DECL(15) = 0;
H_DECL(16) = -S0*S2*magE;
H_DECL(17) = S4;
H_DECL(18) = 0;
H_DECL(19) = 0;
H_DECL(20) = 0;
H_DECL(21) = 0;
H_DECL(22) = 0;
H_DECL(23) = 0;


// Kalman gains
Kfusion(0) = -S9*(P(0,16)*S5 - P(0,17));
Kfusion(1) = -S9*(P(1,16)*S5 - P(1,17));
Kfusion(2) = -S9*(P(2,16)*S5 - P(2,17));
Kfusion(3) = -S9*(P(3,16)*S5 - P(3,17));
Kfusion(4) = -S9*(P(4,16)*S5 - P(4,17));
Kfusion(5) = -S9*(P(5,16)*S5 - P(5,17));
Kfusion(6) = -S9*(P(6,16)*S5 - P(6,17));
Kfusion(7) = -S9*(P(7,16)*S5 - P(7,17));
Kfusion(8) = -S9*(P(8,16)*S5 - P(8,17));
Kfusion(9) = -S9*(P(9,16)*S5 - P(9,17));
Kfusion(10) = -S9*(P(10,16)*S5 - P(10,17));
Kfusion(11) = -S9*(P(11,16)*S5 - P(11,17));
Kfusion(12) = -S9*(P(12,16)*S5 - P(12,17));
Kfusion(13) = -S9*(P(13,16)*S5 - P(13,17));
Kfusion(14) = -S9*(P(14,16)*S5 - P(14,17));
Kfusion(15) = -S9*(P(15,16)*S5 - P(15,17));
Kfusion(16) = -S8*S9;
Kfusion(17) = -S6*S9;
Kfusion(18) = -S9*(P(16,18)*S5 - P(17,18));
Kfusion(19) = -S9*(P(16,19)*S5 - P(17,19));
Kfusion(20) = -S9*(P(16,20)*S5 - P(17,20));
Kfusion(21) = -S9*(P(16,21)*S5 - P(17,21));
Kfusion(22) = -S9*(P(16,22)*S5 - P(17,22));
Kfusion(23) = -S9*(P(16,23)*S5 - P(17,23));


