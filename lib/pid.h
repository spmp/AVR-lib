/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Header file for pid.c.
 *
 * - File:               pid.h
 * - Compiler:           gcc-avr
 * - Supported devices:  All AVR devices can be used.
 * - AppNote:            AVR221 - Discrete PID controller
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support email: avr@atmel.com
 * @author               Jasper Aorangi
 *
 * $Name$
 * $Revision: 900 $
 * $RCSfile$
 * $Date: 2014-07-21$
 *****************************************************************************/

/** @todo move p_factor etc into the PID routine in case we have multiple PID events **/
#pragma once
#include <avr/io.h>
#include <avr/interrupt.h>

#define SCALING_FACTOR  128

/*! \brief PID Status
 *
 * Setpoints and data used by the PID control algorithm
 */
typedef struct PID_DATA{
  //! Last process value, used to find derivative of process value.
  int16_t lastProcessValue;
  //! Summation of errors, used for integrate calculations
  int32_t sumError;
  //! The Proportional tuning constant, multiplied with SCALING_FACTOR
  int16_t P_Factor;
  //! The Integral tuning constant, multiplied with SCALING_FACTOR
  int16_t I_Factor;
  //! The Derivative tuning constant, multiplied with SCALING_FACTOR
  int16_t D_Factor;
  //! Maximum allowed error, avoid overflow
  int16_t maxError;
  //! Maximum allowed sumerror, avoid overflow
  int32_t maxSumError;
} pidData_t;

/*! \brief PID Status
 *
 * Setpoints and data used by the PID control algorithm
 */
typedef struct u_PID_DATA{
  //! Last process value, used to find derivative of process value.
  uint16_t lastProcessValue;
  //! Summation of errors, used for integrate calculations
  int32_t sumError;
  //! The Proportional tuning constant, multiplied with SCALING_FACTOR
  int16_t P_Factor;
  //! The Integral tuning constant, multiplied with SCALING_FACTOR
  int16_t I_Factor;
  //! The Derivative tuning constant, multiplied with SCALING_FACTOR
  int16_t D_Factor;
  //! Maximum allowed error, avoid overflow
  int32_t maxError;
  //! Maximum allowed sumerror, avoid overflow
  int32_t maxSumError;
} u_pidData_t;

/*! \brief Maximum values
 *
 * Needed to avoid sign/overflow problems
 */
// Maximum value of variables
#define MAX_INT         0x7FFF
#define MAX_UINT        0xFFFF
#define MAX_LONG        0x7FFFFFFFL
#define MAX_ULONG       0xFFFFFFFFL
#define MAX_I_TERM      (0x7FFFFFFFL / 2)

// Boolean values
#define FALSE           0
#define TRUE            1

/**
 * @brief Initialise the PID routines
 * @param p_factor, int16_t, the proportional factor
 * @param i_factor, int16_t, the derivative factor
 * @param d_factor, int16_t, the integral factor
 * @param *pid, struct PID_DATA, the pid data 
 * @return none
 **/
void pid_Init(int16_t p_factor, int16_t i_factor, int16_t d_factor, struct PID_DATA *pid);

/**
 * @brief same as above but with uint16_t
 **/
void u_pid_Init(int16_t p_factor, int16_t i_factor, int16_t d_factor, struct u_PID_DATA *pid);

/**
 * @brief PID control algorithm.
 *
 *  Calculates output from setpoint, process value and PID status.
 *
 * @param setPoint,int16,  Desired value.
 * @param processValue, int16,  Measured value.
 * @param *pid_st, struct PID_DATA,  PID status.
 * @return Output update, int16
 */
int16_t pid_Controller(int16_t setPoint, int16_t processValue, struct PID_DATA *pid_st);

/**
 * @brief Same as above but for uint16_t
 **/
int16_t u_pid_Controller(uint16_t setPoint, uint16_t processValue, struct u_PID_DATA *pid_st);

/**
 * @brief Resets the integrator.
 *
 *  Calling this function will reset the integrator in the PID regulator.
 * 
 * @param *pid_st, struct PID_DATA,  PID status.
 */
void pid_Reset_Integrator(pidData_t *pid_st);
