#ifndef BUTTON_H
#define BUTTON_H

#include <rtthread.h>
#include "flexible_button.h"
#include <stdint.h>
#include <rtdevice.h>
#include <board.h>

typedef enum
{
    USER_BUTTON_0 = 0,
    USER_BUTTON_1,
    USER_BUTTON_2,
    USER_BUTTON_MAX
} user_button_t;

#endif
