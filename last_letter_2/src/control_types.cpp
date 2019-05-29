

#include <last_letter_2_msgs/joystick_input.h>
void PD(last_letter_2_msgs::model_states model_states, last_letter_2_msgs::joystick_input channels, last_letter_2_msgs::model_inputs model_inputs)
{
    // if (channels.value[0] == 1500 && channels.value[1] == 1500 && channels.value[3] == 1500)
    //     {
    //         // res.airfoil_inputs[0].x = -1* model_states.base_link_states.roll;
    //         // res.airfoil_inputs[0].y = 5* model_states.base_link_states.pitch;
    //         // res.airfoil_inputs[0].z = model_inputs.wing_input_z[0];
    //     }
    //     else
    //     {
    //         for (i = 0; i < num_wings; i++)
    //         {
    //             res.airfoil_inputs[i].x = model_inputs.wing_input_x[i];
    //             res.airfoil_inputs[i].y = model_inputs.wing_input_y[i];
    //             res.airfoil_inputs[i].z = model_inputs.wing_input_z[i];
    //         }
    //     }

    //     // Motor inputs
    //     for (i = 0; i < num_motors; i++)
    //     {
    //         res.motor_input[i] = model_inputs.motor_input[i];
    //     }
}