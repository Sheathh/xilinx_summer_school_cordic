#include "cordiccart2pol.h"

data_t Kvalues[NO_ITER] = {1,	0.500000000000000,	0.250000000000000,	0.125000000000000,	0.0625000000000000,	0.0312500000000000,	0.0156250000000000,	0.00781250000000000,	0.00390625000000000,	0.00195312500000000,	0.000976562500000000,	0.000488281250000000,	0.000244140625000000,	0.000122070312500000,	6.10351562500000e-05,	3.05175781250000e-05};
// Above: The tan of the cordic method
data_t angles[NO_ITER] = {0.785398163397448,	0.463647609000806,	0.244978663126864,	0.124354994546761,	0.0624188099959574,	0.0312398334302683,	0.0156237286204768,	0.00781234106010111,	0.00390623013196697,	0.00195312251647882,	0.000976562189559320,	0.000488281211194898,	0.000244140620149362,	0.000122070311893670,	6.10351561742088e-05,	3.05175781155261e-05};


data_t positive_rotate(data_t theta_parameter, int j)
{
	// Rotate the axis to accumulate theta (positive direction)
	theta_parameter = theta_parameter + angles[j];
	return theta_parameter;
}

data_t negative_rotate(data_t theta_parameter, int j)
{
	// Rotate the axis to accumulate theta (negative direction)
	theta_parameter = theta_parameter - angles[j];
	return theta_parameter;
}

void cordiccart2pol(data_t x, data_t y, data_t * r,  data_t * theta)// data_t = float
{
	// Write your code here

#pragma HLS INTERFACE mode=s_axilite port=x
#pragma HLS INTERFACE mode=s_axilite port=y
#pragma HLS INTERFACE mode=s_axilite port=theta
#pragma HLS INTERFACE mode=s_axilite port=r
#pragma HLS INTERFACE mode=s_axilite port=return


	data_t k = 0.6072529350088812561694;// The factor that resize cordic result to a proper value (based on rotation time)

	data_t pi_over_2 = 1.570796327;// pi/2, which is used to +/- 90 degrees on the initial angle from the input

	data_t x_value = x;
	data_t y_value = y;
	data_t x_current = 0;
	data_t y_current = 0;// Define the Cartesian coordinate

	data_t theta_acc = 0; // The accumulated angle after all the rotations, it is the ratation of axis!

	if (y < 0)
	{
		theta_acc = theta_acc - pi_over_2;
		x_current = -y_value;
		y_current = x_value; // Rotate for 90 degrees; now still in cartesian
	}
	else
	{
		theta_acc = theta_acc + pi_over_2;
		x_current = y_value;
		y_current = -x_value; // Rotate for 90 degrees; now still in cartesian
	}

	for(int i = 0; i < NO_ITER; i++)
	{
#pragma HLS PIPELINE
		if (y_current >= 0)
		{
			theta_acc = positive_rotate(theta_acc, i); // Determine the new theta
			x_value = x_current + y_current*Kvalues[i];
			y_value = y_current - x_current*Kvalues[i];
		}
		else
		{
			theta_acc = negative_rotate(theta_acc, i); // Determine the new theta
			x_value = x_current - y_current*Kvalues[i];
			y_value = y_current + x_current*Kvalues[i];
		}

		// Now synchronize x and y parameters for the next iteration
		x_current = x_value;
		y_current = y_value;
	}

	// Output the result
	*r = (x_current * k);
	*theta = theta_acc;
}
