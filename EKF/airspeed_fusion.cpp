/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file airspeed_fusion.cpp
 * airspeed fusion methods.
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */
#include "../ecl.h"
#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::fuseAirspeed()
{
	float Hfusion[5];  // Observation Jacobians - Note: indexing is different to state vector
	Vector24f Kfusion; // Kalman gain vector

	const float &vn = _state.vel(0); // Velocity in north direction
	const float &ve = _state.vel(1); // Velocity in east direction
	const float &vd = _state.vel(2); // Velocity in downwards direction
	const float &vwn = _state.wind_vel(0); // Wind speed in north direction
	const float &vwe = _state.wind_vel(1); // Wind speed in east direction

	// Variance for true airspeed measurement - (m/sec)^2
	const float R_TAS = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) *
			    math::constrain(_airspeed_sample_delayed.eas2tas, 0.9f, 10.0f));

	// determine if we need the sideslip fusion to correct states other than wind
	const bool update_wind_only = !_is_wind_dead_reckoning;

	// Intermediate variables
	const float HK0 = vn - vwn;
	const float HK1 = ve - vwe;
	const float HK2 = powf(HK0, 2) + powf(HK1, 2) + powf(vd, 2);
	if (HK2 < 1.0f) {
		// calculation can be badly conditioned for very low airspeed values so don't fuse this time
		return;
	}
	const float v_tas_pred = sqrtf(HK2); // predicted airspeed
	//const float HK3 = powf(HK2, -1.0F/2.0F);
	const float HK3 = 1.0f / v_tas_pred;
	const float HK4 = HK0*HK3;
	const float HK5 = HK1*HK3;
	const float HK6 = 1.0F/HK2;
	const float HK7 = HK0*P(4,6) - HK0*P(6,22) + HK1*P(5,6) - HK1*P(6,23) + P(6,6)*vd;
	const float HK8 = HK1*P(5,23);
	const float HK9 = HK0*P(4,5) - HK0*P(5,22) + HK1*P(5,5) - HK8 + P(5,6)*vd;
	const float HK10 = HK1*HK6;
	const float HK11 = HK0*P(4,22);
	const float HK12 = HK0*P(4,4) - HK1*P(4,23) + HK1*P(4,5) - HK11 + P(4,6)*vd;
	const float HK13 = HK0*HK6;
	const float HK14 = -HK0*P(22,23) + HK0*P(4,23) - HK1*P(23,23) + HK8 + P(6,23)*vd;
	const float HK15 = -HK0*P(22,22) - HK1*P(22,23) + HK1*P(5,22) + HK11 + P(6,22)*vd;
	float HK16;
	 
	// innovation variance
	_airspeed_innov_var = (-HK10*HK14 + HK10*HK9 + HK12*HK13 - HK13*HK15 + HK6*HK7*vd + R_TAS);
	if (_airspeed_innov_var >= R_TAS) { // Check for badly conditioned calculation
		HK16 = HK3 / _airspeed_innov_var;
		_fault_status.flags.bad_airspeed = false;

	} else { // Reset the estimator covariance matrix
		_fault_status.flags.bad_airspeed = true;

		// if we are getting aiding from other sources, warn and reset the wind states and covariances only
		const char* action_string = nullptr;
		if (update_wind_only) {
			resetWindStates();
			resetWindCovariance();
			action_string = "wind";

		} else {
			initialiseCovariance();
			_state.wind_vel.setZero();
			action_string = "full";
		}
		ECL_ERR("airspeed badly conditioned - %s covariance reset", action_string);

		return;
	}

	// Observation Jacobians
	// Note: indexing is different to state vector 
	Hfusion[0] = HK4;    // corresponds to state index 4
	Hfusion[1] = HK5;    // corresponds to state index 5
	Hfusion[2] = HK3*vd; // corresponds to state index 6
	Hfusion[3] = -HK4;   // corresponds to state index 22
	Hfusion[4] = -HK5;   // corresponds to state index 23

	if (update_wind_only) {
		// If we are getting aiding from other sources, then don't allow the airspeed measurements to affect the non-windspeed states
		for (unsigned row = 0; row <= 21; row++) {
			Kfusion(row) = 0.0f;
		}

	} else {
		// we have no other source of aiding, so use airspeed measurements to correct states
		Kfusion(0) = HK16*(-HK0*P(0,22) + HK0*P(0,4) - HK1*P(0,23) + HK1*P(0,5) + P(0,6)*vd);
		Kfusion(1) = HK16*(-HK0*P(1,22) + HK0*P(1,4) - HK1*P(1,23) + HK1*P(1,5) + P(1,6)*vd);
		Kfusion(2) = HK16*(-HK0*P(2,22) + HK0*P(2,4) - HK1*P(2,23) + HK1*P(2,5) + P(2,6)*vd);
		Kfusion(3) = HK16*(-HK0*P(3,22) + HK0*P(3,4) - HK1*P(3,23) + HK1*P(3,5) + P(3,6)*vd);
		Kfusion(4) = HK12*HK16;
		Kfusion(5) = HK16*HK9;
		Kfusion(6) = HK16*HK7;
		Kfusion(7) = HK16*(HK0*P(4,7) - HK0*P(7,22) + HK1*P(5,7) - HK1*P(7,23) + P(6,7)*vd);
		Kfusion(8) = HK16*(HK0*P(4,8) - HK0*P(8,22) + HK1*P(5,8) - HK1*P(8,23) + P(6,8)*vd);
		Kfusion(9) = HK16*(HK0*P(4,9) - HK0*P(9,22) + HK1*P(5,9) - HK1*P(9,23) + P(6,9)*vd);
		Kfusion(10) = HK16*(-HK0*P(10,22) + HK0*P(4,10) - HK1*P(10,23) + HK1*P(5,10) + P(6,10)*vd);
		Kfusion(11) = HK16*(-HK0*P(11,22) + HK0*P(4,11) - HK1*P(11,23) + HK1*P(5,11) + P(6,11)*vd);
		Kfusion(12) = HK16*(-HK0*P(12,22) + HK0*P(4,12) - HK1*P(12,23) + HK1*P(5,12) + P(6,12)*vd);
		Kfusion(13) = HK16*(-HK0*P(13,22) + HK0*P(4,13) - HK1*P(13,23) + HK1*P(5,13) + P(6,13)*vd);
		Kfusion(14) = HK16*(-HK0*P(14,22) + HK0*P(4,14) - HK1*P(14,23) + HK1*P(5,14) + P(6,14)*vd);
		Kfusion(15) = HK16*(-HK0*P(15,22) + HK0*P(4,15) - HK1*P(15,23) + HK1*P(5,15) + P(6,15)*vd);
		Kfusion(16) = HK16*(-HK0*P(16,22) + HK0*P(4,16) - HK1*P(16,23) + HK1*P(5,16) + P(6,16)*vd);
		Kfusion(17) = HK16*(-HK0*P(17,22) + HK0*P(4,17) - HK1*P(17,23) + HK1*P(5,17) + P(6,17)*vd);
		Kfusion(18) = HK16*(-HK0*P(18,22) + HK0*P(4,18) - HK1*P(18,23) + HK1*P(5,18) + P(6,18)*vd);
		Kfusion(19) = HK16*(-HK0*P(19,22) + HK0*P(4,19) - HK1*P(19,23) + HK1*P(5,19) + P(6,19)*vd);
		Kfusion(20) = HK16*(-HK0*P(20,22) + HK0*P(4,20) - HK1*P(20,23) + HK1*P(5,20) + P(6,20)*vd);
		Kfusion(21) = HK16*(-HK0*P(21,22) + HK0*P(4,21) - HK1*P(21,23) + HK1*P(5,21) + P(6,21)*vd);
	}
	Kfusion(22) = HK15*HK16;
	Kfusion(23) = HK14*HK16;


	// Calculate measurement innovation
	_airspeed_innov = v_tas_pred - _airspeed_sample_delayed.true_airspeed;

	// Compute the ratio of innovation to gate size
	_tas_test_ratio = sq(_airspeed_innov) / (sq(fmaxf(_params.tas_innov_gate, 1.0f)) * _airspeed_innov_var);

	// If the innovation consistency check fails then don't fuse the sample and indicate bad airspeed health
	if (_tas_test_ratio > 1.0f) {
		_innov_check_fail_status.flags.reject_airspeed = true;
		return;

	} else {
		_innov_check_fail_status.flags.reject_airspeed = false;
	}

	// Airspeed measurement sample has passed check so record it
	_time_last_arsp_fuse = _time_last_imu;

	// apply covariance correction via P_new = (I -K*H)*P
	// first calculate expression for KHP
	// then calculate P - KHP
	matrix::SquareMatrix<float, _k_num_states> KHP;
	float KH[5];

	for (unsigned row = 0; row < _k_num_states; row++) {

		for (unsigned index = 0; index < 5; index++) {
			KH[index] = Kfusion(row) * Hfusion[index];
		}

		for (unsigned column = 0; column < _k_num_states; column++) {
			float tmp = KH[0] * P(4,column);
			tmp += KH[1] * P(5,column);
			tmp += KH[2] * P(6,column);
			tmp += KH[3] * P(22,column);
			tmp += KH[4] * P(23,column);
			KHP(row,column) = tmp;
		}
	}

	// if the covariance correction will result in a negative variance, then
	// the covariance matrix is unhealthy and must be corrected
	bool healthy = true;
	_fault_status.flags.bad_airspeed = false;

	for (int i = 0; i < _k_num_states; i++) {
		if (P(i,i) < KHP(i,i)) {
			P.uncorrelateCovarianceSetVariance<1>(i, 0.0f);

			healthy = false;

			_fault_status.flags.bad_airspeed = true;

		}
	}

	if (healthy) {
		// apply the covariance corrections
		P -= KHP;

		fixCovarianceErrors(true);

		// apply the state corrections
		fuse(Kfusion, _airspeed_innov);

	}
}

Vector2f Ekf::getWindVelocity() const
{
	return _state.wind_vel;
}

Vector2f Ekf::getWindVelocityVariance() const
{
	return P.slice<2, 2>(22,22).diag();
}

void Ekf::get_true_airspeed(float *tas)
{
	float tempvar = sqrtf(sq(_state.vel(0) - _state.wind_vel(0)) + sq(_state.vel(1) - _state.wind_vel(1)) + sq(_state.vel(2)));
	memcpy(tas, &tempvar, sizeof(float));
}

/*
 * Reset the wind states using the current airspeed measurement, ground relative nav velocity, yaw angle and assumption of zero sideslip
*/
void Ekf::resetWindStates()
{
	const Eulerf euler321(_state.quat_nominal);
	const float euler_yaw = euler321(2);

	if (_tas_data_ready && (_imu_sample_delayed.time_us - _airspeed_sample_delayed.time_us < (uint64_t)5e5)) {
		// estimate wind using zero sideslip assumption and airspeed measurement if airspeed available
		_state.wind_vel(0) = _state.vel(0) - _airspeed_sample_delayed.true_airspeed * cosf(euler_yaw);
		_state.wind_vel(1) = _state.vel(1) - _airspeed_sample_delayed.true_airspeed * sinf(euler_yaw);

	} else {
		// If we don't have an airspeed measurement, then assume the wind is zero
		_state.wind_vel.setZero();
	}
}
