#ifndef _FILTERS_UKF_EXAMPLE_PARAMETERS_HPP
#define _FILTERS_UKF_EXAMPLE_PARAMETERS_HPP

namespace filters_ukf_example
{

struct BlueROV2Parameters
{
    double drag_x = 5.4633;
    double drag_y = 9.4960;
    double drag_z = 14.0424;
    double drag_r = 1.0308;

    double thrust_min = -0.402;
    double thrust_max = 0.515;

    double mass = 12.5;
    double inertia = 0.176;

    double gravity = 9.81;

    struct Sensor2
    {
        double pressure_atm = 101325.0;
        double rho = 1000.0;
    };

    Sensor2 z2;
};

} // namespace filters_ukf_example

#endif // _FILTERS_UKF_EXAMPLE_PARAMETERS_HPP
