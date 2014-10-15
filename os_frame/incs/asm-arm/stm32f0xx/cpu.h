/***************************************************************
 * Name:      cpu.h
 * Purpose:   code for cpu related register file determined by macro in config
 * Author:    mikewang(s)
 * Created:   2014-06-12
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/

#ifndef _CPU_H_
#define _CPU_H_


	#include "config.h"

	#ifdef _STM32F1XXX_

	#include <stm32f10x.h>

	#endif

	#ifdef _STM32F0XXX_

	#include <stm32f0xx.h>

	#endif

#endif


