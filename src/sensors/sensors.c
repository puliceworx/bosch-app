/******************************************************************************
 * @file sensors.c
 *
 * @brief generic sensor implementation
 *
 * Version History
 * ======
 *
 * \addtogroup sensors
 * @{
 *****************************************************************************/

// system includes ------------------------------------------------------------
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>
#include <cmsis_os2.h>
#include <errno.h>

#include <string.h>
#include <stdbool.h>

// local includes -------------------------------------------------------------
#include "worx_task_config.h"
#include "worx_types.h"
#include "main.h"
#include "sensors.h"

// library includes -----------------------------------------------------------

// Macros and Defines ---------------------------------------------------------

LOG_MODULE_REGISTER(APPSENS, CONFIG_APPSENS_LOG_LEVEL);

// enumerations ---------------------------------------------------------------
//static int configure_temp(void)
//{
//  __ASSERT(dev_temp != NULL, "Failed to get device binding");
//  __ASSERT(device_is_ready(dev_temp), "Device %s is not ready", dev_temp->name);
//  LOG_INF("temp device is %p, name is %s", dev_temp, dev_temp->name);
//  return 1;
//}

// structures -----------------------------------------------------------------

// global parameter declarations ----------------------------------------------

// local parameter declarations -----------------------------------------------
const struct device *const dev_temp = DEVICE_DT_GET_ANY(ti_tmp112);

static K_THREAD_STACK_DEFINE(sensorsTask_stack, SENSORS_TASK_STACKSIZE);
//static K_THREAD_STACK_DEFINE(sensorsTask_stack, 2048);
static osThreadAttr_t sensorsTask_attr = {
		.name       = SENSORS_TASK_NAME,
		.stack_mem  = &sensorsTask_stack,
		.stack_size = SENSORS_TASK_STACKSIZE,
    //.stack_size = 2048,
		.priority   = SENSORS_TASK_PRI
};
osThreadId_t sensorsTaskId;

static bool isInit = false;

// local function prototypes --------------------------------------------------
static void sensorsTask(void *param);

bool sensorsInit(void)
{
  // true means there IS an error in initialization
  int res = 0; 
  bool ret = true; 
	struct sensor_value attr;

  if(isInit) 
  {
    return false;
  }

  if (IS_ENABLED(CONFIG_TMP112))
  {
    if (!device_is_ready(dev_temp))
    {
      LOG_ERR("Error: Device %s is not ready", dev_temp->name);
      return ret;
    }


    attr.val1 = 150;
    attr.val2 = 0;
    res = sensor_attr_set(dev_temp, SENSOR_CHAN_AMBIENT_TEMP,
              SENSOR_ATTR_FULL_SCALE, &attr);
    if (res) 
    {
      LOG_ERR("sensor_attr_set failed ret %d", ret);
      return ret;
    }

    attr.val1 = 8;
    attr.val2 = 0;
    res = sensor_attr_set(dev_temp, SENSOR_CHAN_AMBIENT_TEMP,
              SENSOR_ATTR_SAMPLING_FREQUENCY, &attr);
    if (res) 
    {
      LOG_ERR("sensor_attr_set failed ret %d\n", ret);
      return ret;
    }

    ret = false; 
  }

  sensorsTaskId = osThreadNew(sensorsTask, NULL, &sensorsTask_attr);

  isInit = true;
  return ret;
}

static void sensorsTask(void *param)
{
  int ret; 
  struct sensor_value temp_value;

  // Wait till system is ready
  systemWaitStart();

  while (1)
  {
    osDelay(500);
    
    if (IS_ENABLED(CONFIG_TMP112))
    {
  		ret = sensor_sample_fetch(dev_temp);
  		if (ret) 
      {
  			LOG_ERR("sensor_sample_fetch failed ret %d", ret);
  			continue;
  		}

  		ret = sensor_channel_get(dev_temp, SENSOR_CHAN_AMBIENT_TEMP, &temp_value);
  		if (ret) 
      {
  			LOG_ERR("sensor_channel_get failed ret %d", ret);
  			continue;
  		}

  		LOG_INF("temp is %d (%d micro)", temp_value.val1, temp_value.val2);
    }
  }
}

