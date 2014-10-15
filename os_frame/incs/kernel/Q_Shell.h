#ifndef QSYS_Q_SHELL_H  
#define QSYS_Q_SHELL_H

#define USE_Q_SHELL_FUNCTION 1

#if USE_Q_SHELL_FUNCTION

typedef const struct{
	const char*		name;	  //��¼���������	
	const char*		desc;	  //��¼���������
	void           *addr;	  //��¼����ĵ�ַ
	const char*     typedesc; //����¼����Ϊ����ʱ��Ч���������������
}QSH_RECORD;
/*
���ܣ�ע�ắ����������¼����
��Σ�name ������ desc ���������ַ���
���أ���
������
...
unsigned char Var;
QSH_VAR_REG��Var,"unsigned char Var","u8");
...
 */
#define QSH_FUN_REG(name, desc)					                                \
static const   char  qsh_fun_##name##_name[]  = #name;				            \
static const   char  qsh_fun_##name##_desc[]  = desc;						    \
QSH_RECORD qsh_fun_##name##_record  __attribute__((section("qShellFunTab"))) =  \
{							                                                    \
	qsh_fun_##name##_name,	                                                    \
	qsh_fun_##name##_desc,	                                                    \
	(void *)&name,		                                                        \
	0                                                                           \
}
/*
���ܣ�ע�������������¼����
��Σ�name ������ desc ���������ַ��� typedesc �������������ַ���("u8","u16","u32"֮һ)
���أ���
������
...
unsigned char Fun(char *str, unsigned int i, char j)
{
	...
}
QSH_FUN_REG(Fun,"unsigned char Fun(char *str,unsigned int i, char j)");
...
 */
#define QSH_VAR_REG(name, desc,typedesc)					                   \
static const   char  qsh_var_##name##_name[] = #name;				           \
static const   char  qsh_var_##name##_desc[] = desc;				           \
static const   char  qsh_var_##name##_typedesc[] = typedesc;				   \
QSH_RECORD qsh_var_##name##_record  __attribute__((section("qShellVarTab"))) = \
{							                                                   \
	qsh_var_##name##_name,	                                                   \
	qsh_var_##name##_desc,	                                                   \
	(void *)&name,		                                                       \
	qsh_var_##name##_typedesc											       \
}
/* 
���ܣ�shell����Ľӿڣ�ִ������
��Σ�IfCtrl ָʾ�Ƿ���յ�һ�������ַ� CmdStr �Ӵ��ڵõ��������ַ���
���أ�1 �ɹ� 0 ʧ��
�����ַ�����ʽ������
lv()                         �鿴һ��ע������Щ����
get(Var)                     �鿴����Var��ֵ��
set(Var,16)��set(Var,0x10)   ������Var��ֵ
lf()                         �鿴һ��ע������Щ����
fun(��123��,0x20001000,1)    ������ע��ĺ���fun
read(0xE000E004)             �����Ĵ�����ֵ
write(0xE000E004,0xffffffff) ���Ĵ�����ֵ
 */
unsigned int Q_Sh_CmdHandler(unsigned int IfCtrl,char *CmdStr);

#else
/* 
��USE_Q_SHELL_FUNCTION=0��shell���ܱ��رպ�Ϊ�˲���Ҫ�޸�ԭ���������
 */
#define QSH_FUN_REG(name, desc)
#define QSH_VAR_REG(name, desc)
unsigned int Q_Sh_CmdHandler(unsigned int IfCtrl,char *CmdStr);

#endif

#endif
