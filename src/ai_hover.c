/*-----------------------------------------------------------------------------
 File:    ai_hover.c
 Author:  Griffin Bonner      <griffi1@umbc.edu>                           
 Date:    7.22.2021
 Description: hover upon dma interrupt from ai-deck / nn-outputs
-------------------------------------------------------------------------------*/

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "commander.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "uart_dma_setup.h"
#include "log.h"
#include "param.h"

#define BUFFERSIZE 1

uint8_t aideckRxBuffer[BUFFERSIZE];
volatile uint8_t dma_flag = 0;
uint8_t obs_likelihood=0;

/* crazyflie states */
typedef enum {
  idle,
  flying,
  paused,
  stopping
} State;

/* pack setpoint sent to commander */
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;

  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

void appMain()
{
  DEBUG_PRINT("ai-hover application started... \n");
  USART_DMA_Start(115200, aideckRxBuffer, BUFFERSIZE);

  while(1) {
    vTaskDelay(M2T(100));
    if (dma_flag == 1)
      {
	dma_flag = 0;  // clear the flag
	
	// print the inference likelihood of obstacle to debug console
	DEBUG_PRINT("obstacle likelihood: %d\n", aideckRxBuffer[0]);
	obs_likelihood = aideckRxBuffer[0]; // save likehihood
	memset(aideckRxBuffer, 0, BUFFERSIZE);  // clear the dma buffer
      }
  }
}


void __attribute__((used)) DMA1_Stream1_IRQHandler(void)
{
  DMA_ClearFlag(DMA1_Stream1, UART3_RX_DMA_ALL_FLAGS);
  dma_flag = 1;
}

LOG_GROUP_START(log_test)
LOG_ADD(LOG_UINT8, test_variable_x, &obs_likehihood)
LOG_GROUP_STOP(log_test)
