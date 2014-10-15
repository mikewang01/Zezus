#ifndef Task_H_
#define Task_H_



void System_Tick_Hook(void);



#define get_self_taskstate(TASKID,PTASKSTATE)  \
		do{ \
		  PTASKSTATE=OS_get_taskstate();\
		  PTASKSTATE=PTASKSTATE+TASKID;\
		}while(0)

extern u8 Current_Task;



				

				


#endif

