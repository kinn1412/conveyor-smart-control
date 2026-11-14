#ifndef TASKS_H
#define TASKS_H

void app_init(void);  /* creates objects and tasks; call before osKernelStart */
void app_fault(void); /* HardFault, configASSERT, stack overflow: motors off, LED on, halt */

#endif
