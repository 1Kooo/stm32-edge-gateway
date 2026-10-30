#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#include "system.h"

/* Step 1: bring-up entry. Called once from main() after peripherals init. */
void app_main(void);

/* Step 4: create all tasks and start the scheduler. Never returns. */
void App_freeRTOS_start(void);

#endif
