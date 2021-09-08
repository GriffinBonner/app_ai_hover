/*-----------------------------------------------------------------------------
 File:    ai_hover.c
 Author:  Griffin Bonner      <griffi1@umbc.edu>                           
 Date:    7.22.2021
 Description: hover upon dma interrupt from ai-deck / nn-outputs, send network
 inference output back to host pc via real-time logging parameter
-------------------------------------------------------------------------------*/

/* std dependencies */
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>

/* crazyflie dependencies */
#include "app.h"
#include "commander.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "uart_dma_setup.h"
#include "log.h"
#include "param.h"

/* expand for additional network outputs */
#define BUFFERSIZE 5

/* hover altitude in meters */
#define ALTITUDE 0.8

/* obstacle prediction threshold */ 
#define THRESHOLD 0.85

/* filter coefficient */
#define ALPHA 0.85

/* maximum forward velocity */
#define VEL_MAX 0.40

/* time of demonstration (seconds) */
# define RUNTIME 20

/* queue struct to buffer inference data from GAP8 */
struct Queue
{
  uint8_t front, rear, size;
  int sum;
  int capacity;
  uint8_t* array;
}Queue;

/* dynamic x-axis velocity derived from network inference */
float dyn_velocity(float v_k1, float alpha, float inference, float max_velocity)
{
  float unity = 1;
  float v_k;
  v_k = (unity-alpha)*v_k1 + alpha*(unity-inference)*max_velocity;
  return v_k;
}

struct Queue* createQueue(unsigned capacity)
{
  struct Queue* queue = (struct Queue*)malloc(sizeof(struct Queue));
  queue->capacity = capacity;
  queue->front = 0;
  queue->size = 0;
  queue->sum = 0;
  queue->rear = capacity - 1;
  queue->array = (uint8_t*)malloc(capacity*sizeof(uint8_t));
  return queue;
}

/* boolean output queue full */
int isFull(struct Queue* queue)
{
  return (queue->size == queue->capacity);
}

/* boolean output queue empty */
int isEmpty(struct Queue* queue)
{
  return (queue->size == 0);
}

/* add element to the queue & manage sum */
void enqueue(struct Queue* queue, uint8_t element)
{
  if(!isFull(queue)){
    queue->rear = (queue->rear + 1) % queue->capacity;
    queue->array[queue->rear] = element;
    queue->size = queue->size + 1;
    queue->sum = queue->sum + element;
  }else{
    queue->sum = queue->sum - queue->array[queue->front];
    queue->front = (queue->front + 1) % queue->capacity;
    queue->rear = (queue->rear + 1) % queue->capacity;
    queue->array[queue->rear] = element;
    queue->sum = queue->sum + element;
  }
}

/* return oldest element in queue */
uint8_t front(struct Queue* queue)
{
  if (isEmpty(queue))
    return 0;
  return queue->array[queue->front];
}

/* return youngest element in queue */
uint8_t rear(struct Queue* queue)
{
  if (isEmpty(queue))
    return 0;
  return queue->array[queue->rear];
}

/* uart dma parameters */
uint8_t aideckRxBuffer[BUFFERSIZE];
volatile uint8_t dma_flag = 0;


/* inference threshold */
//static const uint8_t inf_threshold = 230; // (255*0.9)

/* application states */
typedef enum {
  idle,
  flying,
  paused,
  stopping
} State;

/* pack hover setpoint sent to commander */
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  /* absolute z-axis distance */
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  
  /* turning (yaw) velocity */ 
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;

  /* velocity in x-axis */
  setpoint->mode.x = modeVelocity;
  setpoint->velocity.x = vx;
  
  /* velocity in y-axis */
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

static State app_state = idle;
static float obstacle_likelihood = 0;

void appMain()
{

  /* start UART DMA */
  DEBUG_PRINT("ai-hover application started... \n");
  USART_DMA_Start(115200, aideckRxBuffer, BUFFERSIZE);
  vTaskDelay(M2T(1000));

  /* create the prediction queue */
  struct Queue* queue = createQueue(BUFFERSIZE);
  
  static setpoint_t setpoint;
  app_state = flying;

  /* takeoff to definied meters in altitude (DEFAULT=0.8) */
  DEBUG_PRINT("Initiating Ascent...\n");
  int i;
  for (i = 1; i <= floor(ALTITUDE*25); i++){
    setHoverSetpoint(&setpoint, 0, 0, 0, 0);
    //setHoverSetpoint(&setpoint, 0, 0, i/25, 0);
    commanderSetSetpoint(&setpoint,4);
    vTaskDelay(M2T(100));
  }
  DEBUG_PRINT("Ascent Complete...\n");

  DEBUG_PRINT("Initiating Trajectory...\n");
  int timesteps = 0;
  while(timesteps <= RUNTIME*100)
  {
    setHoverSetpoint(&setpoint, 0, 0, 0, 0);
    //setHoverSetpoint(&setpoint, 0, 0, ALTITUDE, 0);
    commanderSetSetpoint(&setpoint,4);

    float vx_prev = 0;
    float vx = 0;
    vx += 1;
    vTaskDelay(M2T(10));
    if (dma_flag == 1)
    {
      /* receive & process inference data */
	    dma_flag = 0;                                                     // clear the dma flag
	    DEBUG_PRINT("obstacle likelihood: %d\n", aideckRxBuffer[0]);      // display likelihood
      enqueue(queue, aideckRxBuffer[0]);                                // add network output to inference queue
      obstacle_likelihood = (queue->sum)/(queue->capacity)/(UINT8_MAX); // normalize to float for logging & control
	    memset(aideckRxBuffer, 0, BUFFERSIZE);                            // clear the dma buffer

      /* dynamic velocity control */
      vx = dyn_velocity(vx_prev, ALPHA, obstacle_likelihood, VEL_MAX);  // compute dynamic velocity from inference
      setHoverSetpoint(&setpoint, 0, 0, 0, 0);
      //setHoverSetpoint(&setpoint, vx, 0, ALTITUDE, 0);                  // pack hover setpoint
      commanderSetSetpoint(&setpoint,4);                                // send setpoint to commander
      timesteps += 1;
    }
    timesteps += 1;
  }
}

/* uart dma interrupt-request handler */
void __attribute__((used)) DMA1_Stream1_IRQHandler(void)
{
  DMA_ClearFlag(DMA1_Stream1, UART3_RX_DMA_ALL_FLAGS);
  dma_flag = 1;
}

/* add obstacle likelihood as parameter */
PARAM_GROUP_START(autonomous)
PARAM_ADD(PARAM_FLOAT, obstacle, &obstacle_likelihood)
PARAM_GROUP_STOP(autonomous)

/* add obstacle likelihood as log variable */
//LOG_GROUP_START(log_test)
//LOG_ADD(LOG_UINT8, test_variable_x, &obstacle_likelihood)
//LOG_GROUP_STOP(log_test)
