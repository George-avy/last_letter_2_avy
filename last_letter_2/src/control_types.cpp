
#include <last_letter_2_msgs/joystick_input.h>

void PD(last_letter_2_msgs::model_states &model_states, last_letter_2_msgs::joystick_input &channels, last_letter_2_msgs::model_inputs &model_inputs, float &delta_a, float &delta_e, float &delta_t, float &prev_roll_error, float &prev_pitch_error, float &prev_error, float &dis_alt)
{
    float error, d_error;
    float dt = 0.002;

    error = delta_a - model_states.base_link_states.roll;
    d_error = (error - prev_roll_error) / dt;
    delta_a = 3 * error + 0.5 * d_error;
    if (delta_a < -1)
        delta_a = -1;
    if (delta_a > 1)
        delta_a = 1;

    error = delta_e - model_states.base_link_states.pitch;
    d_error = (error - prev_pitch_error) / dt;
    delta_e = -(2 * error + 0.5 * d_error);
    if (delta_e < -1)
        delta_e = -1;
    if (delta_e > 1)
        delta_e = 1;

    error = dis_alt - model_states.base_link_states.z;
    d_error = (error - prev_error) / dt;
    delta_t=  0.5*error + 0.5* d_error;

    if (delta_t < 0)
        delta_t = 0;
    if (delta_t > 1)
        delta_t = 1;
}