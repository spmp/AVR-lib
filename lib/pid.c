/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief General PID implementation for AVR.
 *
 * Discrete PID controller implementation. Set up by giving P/I/D terms
 * to Init_PID(), and uses a struct PID_DATA to store internal values.
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

#include "pid.h"

/**
 * @brief Initialisation of PID controller parameters.
 *
 * Initialise the variables used by the PID algorithm.
 *
 * @param p_factor  Proportional term.
 * @param i_factor  Integral term.
 * @param d_factor  Derivate term.
 * @param pid  Struct with PID status.
 **/
void pid_Init(int16_t p_factor, int16_t i_factor, int16_t d_factor, struct PID_DATA *pid)
// Set up PID controller parameters
{
  // Start values for PID controller
  pid->sumError = 0;
  pid->lastProcessValue = 0;
  // Tuning constants for PID loop
  pid->P_Factor = p_factor;
  pid->I_Factor = i_factor;
  pid->D_Factor = d_factor;
  // Limits to avoid overflow
  pid->maxError = MAX_INT / (pid->P_Factor + 1);
  pid->maxSumError = MAX_I_TERM / (pid->I_Factor + 1);
}

/**
 * @brief same as above but with uint16_t
 **/
void u_pid_Init(int16_t p_factor, int16_t i_factor, int16_t d_factor, struct u_PID_DATA *pid)
// Set up PID controller parameters
{
  // Start values for PID controller
  pid->sumError = 0;
  pid->lastProcessValue = 0;
  // Tuning constants for PID loop
  pid->P_Factor = p_factor;
  pid->I_Factor = i_factor;
  pid->D_Factor = d_factor;
  // Limits to avoid overflow
  pid->maxError = MAX_LONG / (pid->P_Factor + 1);
  pid->maxSumError = MAX_I_TERM / (pid->I_Factor + 1);
}


/**
 * @brief PID control algorithm.
 *
 * Calculates output from setpoint, process value and PID status.
 *
 * @param setPoint  Desired value.
 * @param processValue  Measured value.
 * @param pid_st  PID status struct.
 **/
int16_t pid_Controller(int16_t setPoint, int16_t processValue, struct PID_DATA *pid_st)
{
  int16_t error, p_term, d_term;
  int32_t i_term, ret, temp;

  error = setPoint - processValue;

  // Calculate Pterm and limit error overflow
  if (error > pid_st->maxError){
    p_term = MAX_INT;
  }
  else if (error < -pid_st->maxError){
    p_term = -MAX_INT;
  }
  else{
    p_term = pid_st->P_Factor * error;
  }

  // Calculate Iterm and limit integral runaway
  temp = pid_st->sumError + error;
  if(temp > pid_st->maxSumError){
    i_term = MAX_I_TERM;
    pid_st->sumError = pid_st->maxSumError;
  }
  else if(temp < -pid_st->maxSumError){
    i_term = -MAX_I_TERM;
    pid_st->sumError = -pid_st->maxSumError;
  }
  else{
    pid_st->sumError = temp;
    i_term = pid_st->I_Factor * pid_st->sumError;
  }

  // Calculate Dterm
  d_term = pid_st->D_Factor * (pid_st->lastProcessValue - processValue);

  pid_st->lastProcessValue = processValue;

  ret = (p_term + i_term + d_term) / SCALING_FACTOR;
  if(ret > MAX_UINT){
    ret = MAX_UINT;
  }
  else if(ret < -MAX_UINT){
    ret = -MAX_INT;
  }

  return((int16_t)ret);
}

/**
 * @brief PID control algorithm.
 *
 * Calculates output from setpoint, process value and PID status.
 *
 * @param setPoint  Desired value.
 * @param processValue  Measured value.
 * @param pid_st  PID status struct.
 **/
int16_t u_pid_Controller(uint16_t setPoint, uint16_t processValue, struct u_PID_DATA *pid_st)
{
    //JA Increase to int32 to contain the maxiumum swing of uint16_t. i.e error must be able to handle +-2^16, p_term must be able to handle +-16*some factor.
    // P_factor can only be as great as 2^15 ie int16
  int32_t error, p_term=0, d_term=0;
  int32_t i_term, ret, temp;

  error = setPoint - processValue;

  // Calculate Pterm and limit error overflow
  if (error > pid_st->maxError){
    p_term = MAX_UINT;
  }
  else if (error < -pid_st->maxError){
    p_term = -MAX_UINT;
  }
  else{
    p_term = pid_st->P_Factor * error;
  }

  // Calculate Iterm and limit integral runaway
  temp = pid_st->sumError + error;
  if(temp > pid_st->maxSumError){
    i_term = MAX_I_TERM;
    pid_st->sumError = pid_st->maxSumError;
  }
  else if(temp < -pid_st->maxSumError){
    i_term = -MAX_I_TERM;
    pid_st->sumError = -pid_st->maxSumError;
  }
  else{
    pid_st->sumError = temp;
    i_term = pid_st->I_Factor * pid_st->sumError;
  }

  // Calculate Dterm
  d_term = pid_st->D_Factor * (pid_st->lastProcessValue - processValue);

  pid_st->lastProcessValue = processValue;

  ret = (p_term + i_term + d_term) / SCALING_FACTOR;
//   if(ret > MAX_INT){
//     ret = MAX_INT;
//   }
//   else if(ret < -MAX_INT){
//     ret = -MAX_INT;
//   }

  return((int16_t)ret);
}

/**
 * @brief Resets the integrator.
 *
 *  Calling this function will reset the integrator in the PID regulator.
 **/
void pid_Reset_Integrator(pidData_t *pid_st)
{
  pid_st->sumError = 0;
}


/**
 * @brief Simple proportional control algorythm.
 * @param Measured: The measured value
 * @param SetPoint: The setpoint to be optained from measurement
 * @param MeasuredMax: The maxium measured value
 * @param OutputMax: The maxium output value
 * @param Proportionality: The proportional quantity modifying the difference
 * @param Power: The power to which the difference is raised. Must be an odd number! (i.e. 1 or 3)
 * @return The calculated addition to the Output quantity
 * 
 * @description 
 * This algorythom works as follows
 *  OUTPUT = (Proportionality * Gamma * (SetPoint - Measured ))^Power
 *  Where Gamma is a linear transfer function for the range of measured values VS the range out Output Values such that they are normaliesd.
 *  In terms of unit analysis, we want OUTPUT to be of type OUTPUT, so we require Gamma = OutputMax*MeasuredMax 
 **/
int32_t pid_proportional( int32_t Measured, int32_t SetPoint, int32_t MeasuredMax, int32_t OutputMax, float Proportionality, uint8_t Power){
    float Gamma;
    Gamma = OutputMax / MeasuredMax;
    
    return pow(Proportionality*Gamma*(SetPoint-Measured),Power);
}

/**
 * @brief Simple proportional control algorythm.
 * @param Measured: The measured value
 * @param SetPoint: The setpoint to be optained from measurement
 * @param MeasuredMax: The maxium measured value
 * @param OutputMax: The maxium output value
 * @param Proportionality: The proportional quantity modifying the difference
 * @param Power: The power to which the difference is raised. Must be an odd number! (i.e. 1 or 3)
 * @param Max: The maximum outut
 * @return The calculated addition to the Output quantity
 * 
 * @description 
 * This algorythom works as follows
 *  OUTPUT = (Proportionality * Gamma * (SetPoint - Measured ))^Power
 *  Where Gamma is a linear transfer function for the range of measured values VS the range out Output Values such that they are normaliesd.
 *  In terms of unit analysis, we want OUTPUT to be of type OUTPUT, so we require Gamma = OutputMax*MeasuredMax 
 **/
int32_t pid_proportional_max( int32_t Measured, int32_t SetPoint, int32_t MeasuredMax, int32_t OutputMax, float Proportionality, uint8_t Power, int32_t max ){
    float Gamma;
    Gamma = OutputMax / MeasuredMax;
    
    return max(min(pow(Proportionality*Gamma*(SetPoint-Measured),Power), max), -max);
}


/**
 * @brief Simple proportional control algorythm V2 for large value input
 * @param Measured: The measured value
 * @param SetPoint: The setpoint to be optained from measurement
 * @param Proportionality: The proportional quantity modifying the difference
 * @return The calculated addition to the Output quantity
 **/
int32_t pid_proportional_current( int32_t Measured, int32_t SetPoint, int16_t Proportion, int16_t MinMax){
    int16_t delta;
    delta = (SetPoint-Measured)/Proportion;
    if (delta < -MinMax)
        return -MinMax;
    else if (delta >= MinMax)
            return MinMax;
    else 
        return  delta;
}
