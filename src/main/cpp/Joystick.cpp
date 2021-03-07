#include "Joystick.h"
#include <stdio.h>
#include <iostream>

#define TRACKWIDTH 0.61f
#define HALF_TRACKWIDTH (TRACKWIDTH / 2.0f)

#define AMAX 5.1 // Acceleration Max  au PIF .. à définir aux encodeurs
#define VMAX 3.4 // vitesse Max  théorique (3,395472 sur JVN-DT) .. à vérifier aux encodeurs
#define WMAX                       \
    (((2.0 * VMAX) / TRACKWIDTH) / \
     1.7) // vitesse angulaire Max theorique	.. à modifier avec Garice

// Flags Manipulation
#define FLAG_TOGGLE(val, flag) ((val) ^= (flag))
#define FLAG_ON(val, flag) ((val) |= (flag))
#define FLAG_OFF(val, flag) ((val) &= ~(flag))
#define ISFLAG_ON(val, flag) ((val) & (flag))                                   // !! ZERO or NON ZERO value !!! BE AWARE THAT NON ZERO DOESN'T MEAN 1 !!!
#define ISFLAG_OFF(val, flag) (!((val) & (flag)))                               // !! ZERO or NON ZERO value !!! BE AWARE THAT NON ZERO DOESN'T MEAN 1 !!!
#define FLAGS_TEST(val, bmask, flags) (((val) & (bmask)) == (flags))            // NFALSE or NTRUE
#define SET_FLAGS(val, bmask, flags) ((val) = (((val) & (~(bmask))) | (flags))) // RESET FLAGS BITS and Set them all like flags.
#define RESET_FLAGS(val, bmask)		((val)&=~(bmask)))						// Set all FLAGS BITS to ZERO

#define VOLTAGE_COMPENSATION_VALUE 11.5
// TEST *********************************************
#define TEST_LOWVOLTAGE_NB 10    // Nombre de tests ( subdivisions ) sur l'intervalle ]0,TEST_LOWVOLTAGE_MAX] volts						... 10 ou 20 ?
#define TEST_LOWVOLTAGE_MAX 0.15 // Volts

#define TEST_MEDIUMVOLTAGE_NB 5    // Nombre de tests ( subdivisions ) sur l'intervalle ]TEST_LOWVOLTAGE_MAX,TEST_MEDIUMVOLTAGE_MAX] volts	... 20 ou 25 ?
#define TEST_MEDIUMVOLTAGE_MAX 1.0 // Volts

#define TEST_HIGHVOLTAGE_NB 44    // Nombre de tests ( subdivisions ) sur l'intervalle ]TEST_MEDIUMVOLTAGE_MAX,TEST_HIGHVOLTAGE_MAX] volts... 12 ou 24 ?
#define TEST_HIGHVOLTAGE_MAX 12.0 // Volts

#define TEST_TOTAL_NB (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + TEST_HIGHVOLTAGE_NB)

#define FLAG_TestSpecs_Done 1

#define TIME_RAMP 0.6

#define EPSILON 0.0000001

// Differential Steering Joystick Algorithm
// ========================================
// Converts a single dual-axis joystick into a differential
// drive motor control, with support for both drive, turn
// and pivot operations.
//
double getSign(double number)
{
    if (number < 0)
    {
        return -1;
    }
    else
    {
        return 1;
    }
}
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
    double blend_threshold = 0.5;

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
}

void getSpeedsAndAccelerationsNew(VA *pva_left, VA *pva_right, const VA *pva_max, const double jx, const double jy)
{
    double target_left_speed;
    double target_right_speed;

    double v = jx * VMAX;
    double w = jy * WMAX;

    // w = m_drivetrain->CalculateTurn(forward, w);

    double lwheel = v + (w * HALF_TRACKWIDTH);
    double rwheel = v - (w * HALF_TRACKWIDTH);

    double k;
    k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
    lwheel *= k;
    rwheel *= k;

    target_left_speed = lwheel * VMAX;
    target_right_speed = rwheel * VMAX;

    updateVelocityAndAcceleration(pva_left, pva_max, target_left_speed, 0.02);
    updateVelocityAndAcceleration(pva_right, pva_max, target_right_speed, 0.02);
}

void updateVelocityAndAcceleration(VA *pva, const VA *pva_max, const double target_speed, const double dt)
{
    double dv0v1 = target_speed - pva->m_speed;
    double dv_a = getSign(pva->m_acceleration) * pva->m_acceleration * pva->m_acceleration / (2.0f * pva_max->m_jerk);
    double d_v = dv0v1 - dv_a;

    if (d_v < 0)
    {
        if (pva->m_acceleration <= -pva_max->m_acceleration)
        {
            pva->m_jerk = 0.0;
        }
        else
        {
            pva->m_jerk = -pva_max->m_jerk;
        }
    }
    else if (d_v > 0)
    {
        if (pva->m_acceleration >= pva_max->m_acceleration)
        {
            pva->m_jerk = 0.0;
        }
        else
        {
            pva->m_jerk = pva_max->m_jerk;
        }
    }
    else
    {
        pva->m_jerk = 0.0f;
    }

    double a = pva->m_acceleration + pva->m_jerk * dt;

    if (a > pva_max->m_acceleration)
    {
        a = pva_max->m_acceleration;
    }
    else if (a < -pva_max->m_acceleration)
    {
        a = -pva_max->m_acceleration;
    }

    double t = abs(a - pva->m_acceleration) / pva_max->m_jerk;

    double da = pva->m_acceleration * t + 0.5 * t * t * pva->m_jerk + a * (dt - t);
    if (getSign(dv0v1) != getSign(dv0v1 - da))
    {
        pva->m_acceleration = 0.0;
        pva->m_jerk = 0.0;
        pva->m_speed = target_speed;
    }
    else
    {
        pva->m_speed += da;
        pva->m_acceleration = a;
    }
    //std::cout << pva->m_speed << std::endl;
    /*
    if (dv0v1 < 0)
    {
        if (dv0v1 < getSign(pva->m_acceleration) * pva->m_acceleration * pva->m_acceleration / (2.0f * pva_max->m_jerk))
        {
            pva->m_acceleration -= pva_max->m_jerk * dt;
            if (pva->m_acceleration < -pva_max->m_acceleration)
            {
                std::cout << "dommage" << std::endl;
                pva->m_acceleration = -pva_max->m_acceleration;
            }
        }
        else
        {
            pva->m_acceleration += pva_max->m_jerk * dt;
        }
    }
    else if (dv0v1 > 0)
    {
        if (dv0v1 > getSign(pva->m_acceleration) * pva->m_acceleration * pva->m_acceleration / (2.0f * pva_max->m_jerk))
        {
            pva->m_acceleration += pva_max->m_jerk * dt;
            if (pva->m_acceleration > pva_max->m_acceleration)
            {
                std::cout << "banane" << std::endl;
                pva->m_acceleration = pva_max->m_acceleration;
            }
        }
        else
        {
            pva->m_acceleration -= pva_max->m_jerk * dt;
        }
    }
    else
    {
        //std::cout << pva->m_acceleration << "    alors qu'elle devrait être nulle" << std::endl;
        return;
    }

    pva->m_speed += pva->m_acceleration * dt;
    std::cout << pva->m_acceleration << "     " << pva->m_speed << "     " << target_speed << "     " << dv0v1 << std::endl;*/
}