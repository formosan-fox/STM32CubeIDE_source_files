/*
 * package.cpp
 *
 *  Created on: Aug 16, 2023
 *      Author: a9163
 */

#include "package.h"
#include "util.h"
#include "math.h"

EvaluationWindow<float> para_eva(GET_DOWNTREND_SIZE, [](float x, float y) { return x < y; });
EvaluationWindow<float> depl_eva(START_DEPLOY_TIMER_SIZE, [](float x, float y) { return x > y; });
EvaluationWindow<float>  liftoff_eva(LIFTOFF_SIZE, [](float x, float y) { return x > y; });
EvaluationWindow<float>  separation_eva(SEPARATION_SIZE, [](float x, float y) { return x < y; });

// Ignore the preceding data (incorrect data)
void Takeoff_separation_parachute_judgmental::StartCorrectData(bool *DataReady, bool *breakLoop)
{

        if (*DataReady == true)
        {
            start_read_number ++;
            *DataReady = false;
        }
        if (start_read_number == 200)
        {
            *breakLoop = true;
        }
    
}

// Get the horizontal line altitude 
void Takeoff_separation_parachute_judgmental::Initilize_altitude(bool *DataReady, float ALTITUDE, bool *breakLoop)
{

        if (*DataReady == true && initial_altitude_number < GET_STD_SIZE)
        {
            initial_altitude[initial_altitude_number] = ALTITUDE;
            initial_altitude_number ++;
            *DataReady = false;
        }
        if (initial_altitude_number == GET_STD_SIZE)
        {
            initial_altitude_value = Takeoff_separation_parachute_judgmental::Initilize_getSTD();
            std_altitude = initial_altitude_value + 50;
            *breakLoop = true;
        }
    
}

float Takeoff_separation_parachute_judgmental::Initilize_getSTD()
{
    float sum = 0.0;
    float std = 0.0;
    float variance = 0.0;
    float valid_elements = GET_STD_SIZE;
    int i;
    
    for (i = 0; i < valid_elements; i++)
    {
        sum += initial_altitude[i];
    }
    initial_altitude_value = sum / valid_elements;
    for (i = 0; i < valid_elements; i++)
    {
        variance += powf(initial_altitude[i] - initial_altitude_value, 2);
     }
    std = sqrtf(variance / 10);
    float max = initial_altitude_value + 2 * std, min = initial_altitude_value - 2 * std;
    for (i = 0; i < valid_elements; i++)
    {
        if (initial_altitude[i] < min || initial_altitude[i] > max)
        {
            valid_elements --;
            sum -= initial_altitude[i];
    }
  }
  initial_altitude_value = sum / valid_elements;
  return initial_altitude_value;
}

// start deploy timer by determing yacc > 2000
void  Takeoff_separation_parachute_judgmental::StartDeployNumber(bool *DataReady, float yacc, bool *breakLoop)
{
    if (*DataReady == true){
      depl_eva.updateData(yacc, 2000);
      // 65/80 
      if (depl_eva.getSuccessNumber() >= DEPLOY_TIMER_TARGET)
      {
         *breakLoop = true;
      }
      *DataReady = false;
    }

}

// liftoff when altitude > initial altitude + 50
void  Takeoff_separation_parachute_judgmental::CheckTheRocketIsTakeoff(bool *DataReady, float altitude, bool *breakLoop)
{
    if (*DataReady == true){
      liftoff_eva.updateData(altitude, std_altitude);
      // 9/10
      if (liftoff_eva.getSuccessNumber() >= LIFTOFF_TARGET)
      {
         *breakLoop = true;
      }
      *DataReady = false;
    }
}

// separation after yacc < 0
void  Takeoff_separation_parachute_judgmental::CheckTheRocketCanSeparation(bool *DataReady, float yacc, bool *breakLoop)
{
    if (*DataReady == true){
      separation_eva.updateData(yacc, -0);
      // 18/20
      if (separation_eva.getSuccessNumber() >= SEPARATION_TARGET)
      {
         *breakLoop = true;
      }
        *DataReady = false;
    }
}

// altitude values are downtrend
void  Takeoff_separation_parachute_judgmental::CheckTheRocketCanDeploy(bool *DataReady, float altitude, bool *breakLoop, int DeployNumber)
{
    if (*DataReady == true){
      cur_WSEN_data = altitude;
      para_eva.updateData(cur_WSEN_data, pre_WSEN_data);
      pre_WSEN_data = cur_WSEN_data;
      // 45/50 
      if (para_eva.getSuccessNumber() >= DOWNTREND_TARGET || DeployNumber >= 12460)
      {
         *breakLoop = true;
      }
      *DataReady = false;
    }
}

// use timer to read altitude
void  Takeoff_separation_parachute_judgmental::Timer2GetAltitude(bool *DataReady, float *altitude)
{
        moving_avg_altitude = moving_avg_altitude - altitude_avg[timer_altitude_count];
        altitude_avg[timer_altitude_count] = *altitude / 5;
        moving_avg_altitude = moving_avg_altitude + altitude_avg[timer_altitude_count];
        *altitude = moving_avg_altitude;
        timer_altitude_count ++;

        if (timer_altitude_count == 5){
        	timer_altitude_count = 0;
        }
        *DataReady = true;
}

// use timer to read yacc
void  Takeoff_separation_parachute_judgmental::Timer2GetYacc(bool *DataReady, float *yacc)
{
        moving_avg_yacc = moving_avg_yacc - yacc_avg[timer_yacc_count];
        yacc_avg[timer_yacc_count] = *yacc / 5;
        moving_avg_yacc = moving_avg_yacc + yacc_avg[timer_yacc_count];
        *yacc = moving_avg_yacc;
        timer_yacc_count ++;

        if (timer_yacc_count == 5){
        	timer_yacc_count = 0;
        }
        *DataReady = true;
}

// use timer to count time
void  Takeoff_separation_parachute_judgmental::Timer2GetDeployTime(bool DeployReady, int *DeployNumber)
{
    if (DeployReady == true)
    {
        (*DeployNumber) ++;
    }
}







