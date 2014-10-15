#ifndef __ADC_H
#define __ADC_H

//ADC_CLOCK
#define ADC_CLK_36	0
#define ADC_CLK_18	1
#define ADC_CLK_12	2
#define ADC_CLK_9	3	

//ADC->SR
#define STRT 	4	//����ͨ����ʼλ
#define JSTRT	3	//ע��ͨ����ʼλ
#define JEOC	2	//ע��ͨ��ת������λ
#define EOC		1	//����ͨ��ת������λ
#define AWD		0	//ģ�⿴�Ź���־λ

//ADC->CR1
#define AWDEN	23	//����ͨ�����Ź�ʹ��
#define	JWDEN	22	//ע��ͨ�����Ź�ʹ��
#define MODE	16	//ģʽѡ��,0Ϊ����ģʽ
#define SCAN	8	//ɨ��ģʽʹ��λ
#define JECOIE	7	//ע��ͨ��ת�������ж�ʹ��λ
#define AWDIE	6	//ģ�⿴�Ź��ж�ʹ��λ
#define EOCIE	5	//EOC�ж�ʹ��λ
#define AWDCH	0	//ģ�⿴�Ź�ͨ��ѡ��λ(4:0)

//ADC->CR2
#define	SWSTART		22	//��ʼת������ͨ��
#define	JSWSTART	21	//��ʼת��ע��ͨ��
#define	EXTRIG		20	//����ͨ���ⲿ����ģʽʹ��λ
#define	EXTSEL		17	
#define ALIGN		11	//���ݶ���
#define ADC_DMA		8	//DMAʹ��λ
#define RSTCAL		3	//��λУ׼
#define CAL			2	//ADУ׼
#define CONT		1	//����ת��
#define	ADON		0	//ADCʹ��λ



void ADC1_Init(void);
void ADC_int(void);

void MYDMA_Enable(void);
#endif
