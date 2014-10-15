 //----------------------------------------------------------------------------
// WSH Corp. Company Confidential Strictly Private
//
// $RCSfile: task.c,v $
// $Revision: 0.1 $
// $Author: Mike Wang $
// $Date: 2014/5/10 19:37:23 $
// $Description: Task file
//
// ---------------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<
// ---------------------------------------------------------------------------
// Copyright 2013-2014 (c) WSH Corp.
//
// WSH Corp. owns the sole copyright to this software. Under international
// copyright laws you (1) may not make a copy of this software except for
// the purposes of maintaining a single archive copy, (2) may not derive
// works herefrom, (3) may not distribute this work to others. These rights
// are provided for information clarification, other restrictions of rights
// may apply as well.
//
// This is an unpublished work.
// ---------------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>>>>> WARRANTEE <<<<<<<<<<<<<<<<<<<<<
// ---------------------------------------------------------------------------
// WSH Corp. MAKES NO WARRANTY OF ANY KIND WITH REGARD TO THE USE OF
// THIS SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR
// PURPOSE.
// ---------------------------------------------------------------------------


#ifndef __CLKMNG_H__
#define __CLKMNG_H__

#include "kernel-includes.h"
#include "drivers-includes.h"
enum clock_status{clock_closed=0,clock_opened};

typedef struct
{
		struct os_object parent;
	  u8 status;
	  u8 last_status;
	  os_err_t (*clock_enable)(void);
    os_err_t (*clock_disable)(void);
	#ifdef CLOCK_IN_ROM
	 status_record *pstatus_record;
	#endif
} Peripheral_clock;


enum {sleepmode,stopmode,standbymode};

os_err_t os_clock_register(Peripheral_clock *clock,   
                            const char  *name 
                           ); 
os_err_t os_clock_open(const char * name);
os_err_t os_clock_close(const char * name);
os_err_t os_clock_restore(const char * name);	


#endif // 

