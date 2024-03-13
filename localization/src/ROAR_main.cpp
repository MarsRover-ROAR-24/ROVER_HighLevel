#include "ROAR_UKF.h"

int main() {

	double encoder_inputs = 0;
	double dt = 0.1;


	MerwedSigmaPoints::sigma_points(9, 3.0, 2.0, 0.1);
	UKF::ROAR_UKF(sigma_points);
	// Prediction Step
	UKF::prediction(encoder_inputs, dt)


	return 0;
}