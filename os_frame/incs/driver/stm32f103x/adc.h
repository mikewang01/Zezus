#ifndef __ADC_H
#define __ADC_H

//ADC_CLOCK
#define ADC_CLK_36	0
#define ADC_CLK_18	1
#define ADC_CLK_12	2
#define ADC_CLK_9	3	

//ADC->SR
#define STRT 	4	//规则通道开始位
#define JSTRT	3	//注入通道开始位
#define JEOC	2	//注入通道转换结束位
#define EOC		1	//规则通道转换结束位
#define AWD		0	//模拟看门狗标志位

//ADC->CR1
#define AWDEN	23	//规则通道看门狗使能
#define	JWDEN	22	//注入通道看门狗使能
#define MODE	16	//模式选择,0为独立模式
#define SCAN	8	//扫描模式使能位
#define JECOIE	7	//注入通道转换结束中断使能位
#define AWDIE	6	//模拟看门狗中断使能位
#define EOCIE	5	//EOC中断使能位
#define AWDCH	0	//模拟看门狗通道选择位(4:0)

//ADC->CR2
#define	SWSTART		22	//开始转换规则通道
#define	JSWSTART	21	//开始转换注入通道
#define	EXTRIG		20	//规则通道外部触发模式使能位
#define	EXTSEL		17	
#define ALIGN		11	//数据对齐
#define ADC_DMA		8	//DMA使能位
#define RSTCAL		3	//复位校准
#define CAL			2	//AD校准
#define CONT		1	//连续转换
#define	ADON		0	//ADC使能位



void ADC1_Init(void);
void ADC_int(void);

void MYDMA_Enable(void);
#endif
