#include "Joystick.h"
#include <stdio.h>
#include <iostream>

// Differential Steering Joystick Algorithm
// ========================================
// Converts a single dual-axis joystick into a differential
// drive motor control, with support for both drive, turn
// and pivot operations.
//

void getSpeedsAndAccelerations(VA *pva_left, VA *pva_right, const VA *pvamax, const double jx, const double jy)
{
    double premix_left;  //    (left) premixed output(-1.. + 1)
    double premix_right; //    (right)premixed output(-1.. + 1)

    double omega; //    pivot speed
    double blend;

    // blend_threshold  : The threshold at which the pivot action starts
    //                This threshold is measured in units on the Y-axis
    //                away from the X-axis (Y=0). A greater value will assign
    //                more of the joystick's range to pivot actions.
    //                Allowable range: (0..1)
    double blend_threshold = 0.75;

    if (jy >= 0)
    {
        // Forward
        premix_left = (jx >= 0.0) ? 1.0 : (1.0 + jx);
        premix_right = (jx >= 0.0) ? (1.0 - jx) : 1.0;
    }
    else
    {
        // Reverse
        premix_left = (jx >= 0.0) ? (1.0 - jx) : 1.0;
        premix_right = (jx >= 0.0) ? 1.0 : (1.0 + jx);
    }

    // Scale Drive output due to Joystick Y input (throttle)
    premix_left *= jy;
    premix_right *= jy;

    // Now calculate pivot amount
    // - Strength of pivot (nPivSpeed) based on Joystick X input
    // - Blending of pivot vs drive (blend) based on Joystick Y input
    omega = jx;
    blend = (NABS(jy) > blend_threshold) ? 0.0 : (1.0 - (NABS(jy) / blend_threshold));
    std::cout << blend << std::endl;
    double mix_left;
    double mix_right;

    // Calculate final mix of Drive and Pivot
    mix_left = (1.0 - blend) * premix_left + blend * (omega);
    mix_right = (1.0 - blend) * premix_right + blend * (-omega);

    double target_left_speed;
    double target_right_speed;

    target_left_speed = mix_left * pvamax->m_speed;
    target_right_speed = mix_right * pvamax->m_speed;

    double acc;
    double v_diff;

    //Left side
    acc = pvamax->m_acceleration * 0.02;
    v_diff = target_left_speed - pva_left->m_speed;

    if (v_diff < -acc)
    {
        pva_left->m_speed -= acc;
        pva_left->m_acceleration = pvamax->m_acceleration;
    }
    else if (v_diff > acc)
    {
        pva_left->m_speed += acc;
        pva_left->m_acceleration = pvamax->m_acceleration;
    }
    else
    {
        pva_left->m_speed = target_left_speed;
        pva_left->m_acceleration = 0;
    }

    //Right side
    acc = pvamax->m_acceleration * 0.02;
    v_diff = target_right_speed - pva_right->m_speed;

    if (v_diff < -acc)
    {
        pva_right->m_speed -= acc;
        pva_right->m_acceleration = pvamax->m_acceleration;
    }
    else if (v_diff > acc)
    {
        pva_right->m_speed += acc;
        pva_right->m_acceleration = pvamax->m_acceleration;
    }
    else
    {
        pva_right->m_speed = target_right_speed;
        pva_right->m_acceleration = 0;
    }
    std::cout << "vitesse : " << pva_right->m_speed << "   " << pva_left->m_speed << std::endl;
}