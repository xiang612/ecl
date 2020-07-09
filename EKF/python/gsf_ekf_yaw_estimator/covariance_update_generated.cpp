// Equations for covariance matrix update
const float S0 = P11 + velObsVar;
const float S1 = powf(P01, 2);
const float S2 = -S1;
const float S3 = P00 + velObsVar;
const float S4 = S0*S3;
const float S5 = S2 + S4;
const float S6 = 1.0F/S5;
const float S7 = S6*velObsVar;
const float S8 = (-P00*S0 + S1)/(S1 - S4);
const float S9 = S0*S7 + S8;
const float S10 = S1*velObsVar;
const float S11 = S10*S6;
const float S12 = S11 + S3*S8;
const float S13 = P11*S3;
const float S14 = S1 - S13;
const float S15 = S6*S9;
const float S16 = P01*P02 - P12*S3;
const float S17 = P01*S16;
const float S18 = P01*P12 - P02*S0;
const float S19 = powf(S5, -2);
const float S20 = S19*(-S0*S14 + S10);
const float S21 = S13 + S2 + S3*velObsVar;
const float S22 = S6*S8;
const float S23 = S19*S21;
const float S24 = P01*S18;
const float S25 = S19*(S0*S16 + S24);
const float S26 = P01*velObsVar;
const float S27 = S17 + S18*S3;
const float S28 = S19*S27;


_ekf_gsf[model_index].P(0,0) = P00 - S11*S9 - S12*S8;
_ekf_gsf[model_index].P(0,1) = P01*(-S12*S7 + S14*S15 + 1);
_ekf_gsf[model_index].P(1,1) = P11 - S10*S23 + S14*S20;
_ekf_gsf[model_index].P(0,2) = P02 + S12*S18*S6 + S15*S17;
_ekf_gsf[model_index].P(1,2) = P12 + S16*S20 + S23*S24;
_ekf_gsf[model_index].P(2,2) = P22 - S16*S25 - S18*S28;


