#include "Q_Shell.h"
#include "string.h"
#include "stdio.h"
#include "kernel-includes.h"

#if USE_Q_SHELL_FUNCTION      
//»·¾³±äÁ¿²»Õ¼ÓÃÄÚ´æ
extern int qShellFunTab$$Base;	//º¯Êı¼ÇÂ¼¶Î¶ÎÊ×
extern int qShellFunTab$$Limit;	//º¯Êı¼ÇÂ¼¶Î¶ÎÎ²
extern int qShellVarTab$$Base;	//±äÁ¿¼ÇÂ¼¶Î¶ÎÊ×
extern int qShellVarTab$$Limit;	//±äÁ¿¼ÇÂ¼¶Î¶ÎÎ²

#define QSH_FUN_PARA_MAX_NUM 4//º¯Êı×î¶àÖ§³ÖËÄ¸ö²ÎÊı
#define SHELL_CMD_SIZE 64

typedef unsigned int (*QSH_FUNC_TYPE)();//Í³Ò»µÄshellº¯ÊıÀàĞÍÖ§³Ö²»Í¬¸öÊıµÄ²ÎÊı


struct finsh_shell
{
    u8 esc_flag;
    u8 escbuf[4];
    char line[SHELL_CMD_SIZE];
    u8 line_position;
    u8 line_curpos;
    u8 status;
    u8 ifpass;
    
};

enum input_stat
{
    WAIT_NORMAL,
    WAIT_SPEC_KEY,
    WAIT_FUNC_KEY,
};

typedef enum{
    QSCT_ERROR,        //ÎŞĞ§µ÷ÓÃ
    QSCT_LIST_FUN,     //ÁĞ³öËùÓĞ¿É±»shell·ÃÎÊµÄº¯Êı
    QSCT_HELP_FUN,     //ÁĞ³öËùÓĞ¿É±»shell·ÃÎÊµÄº¯Êı
    QSCT_LIST_VAR,     //ÁĞ³öËùÓĞ¿É±»shell·ÃÎÊµÄ±äÁ¿
    QSCT_CALL_FUN,     //µ÷ÓÃº¯Êı
    QSCT_READ_VAR,     //²é¿´±äÁ¿µÄÖµ
    QSCT_WRITE_VAR,    //¸ø±äÁ¿¸³Öµ
    QSCT_READ_REG,     //¶Á¼Ä´æÆ÷
    QSCT_WRITE_REG     //Ğ´¼Ä´æÆ÷
}Q_SH_CALL_TYPE;       //shellµ÷ÓÃÀàĞÍ

typedef enum{
    QSFPT_NOSTRING,   //·Ç×Ö·û´®ĞÍ²ÎÊı
    QSFPT_STRING      //×Ö·û´®ĞÍ²ÎÊı
}Q_SH_FUN_PARA_TYPE;  //º¯Êı²ÎÊıÀàĞÍ

typedef struct{
    char               *CmdStr;    //ÃüÁî×Ö·û´®Ê×µØÖ·
    Q_SH_CALL_TYPE     CallType;   //shellµ÷ÓÃÀàĞÍ
    unsigned char      CallStart;  //shellµ÷ÓÃÃûÔÚÃüÁî×Ö·û´®ÖĞµÄÆğÊ¼Î»ÖÃ                   
    unsigned char      CallEnd;    //shellµ÷ÓÃÃûÔÚÃüÁî×Ö·û´®ÖĞµÄ½áÊøÎ»ÖÃ 
    unsigned char      ParaNum;    //shellµ÷ÓÃµÄ²ÎÊı¸öÊı
    unsigned char      ParaStart[QSH_FUN_PARA_MAX_NUM];//Ã¿¸ö²ÎÊıÔÚÃüÁî×Ö·û´®ÖĞµÄÆğÊ¼Î»ÖÃ   
    unsigned char      ParaEnd[QSH_FUN_PARA_MAX_NUM];  //Ã¿¸ö²ÎÊıÔÚÃüÁî×Ö·û´®ÖĞµÄ½áÊøÎ»ÖÃ   
    Q_SH_FUN_PARA_TYPE ParaTypes[QSH_FUN_PARA_MAX_NUM];//Ã¿¸ö²ÎÊıµÄÀàĞÍ               
}Q_SH_LEX_RESU_DEF;



static struct finsh_shell shell={0};
/* 
¹¦ÄÜ£ºÔÚº¯Êı¼ÇÂ¼¶ÎÑ°ÕÒÖ¸¶¨µÄº¯Êı
Èë²Î£ºº¯ÊıÃû
·µ»Ø£º¶ÔÓ¦µÄº¯Êı¼ÇÂ¼
 */
static QSH_RECORD* Q_Sh_SearchFun(char* Name)
{
    QSH_RECORD* Index;
    for (Index = (QSH_RECORD*) &qShellFunTab$$Base; Index < (QSH_RECORD*) &qShellFunTab$$Limit; Index ++)
    {
        if (strcmp(Index->name, Name) == 0)
            return Index;
    }
    return (void *)0;
}
/* 
¹¦ÄÜ£ºÔÚº¯Êı¼ÇÂ¼¶ÎÑ°ÕÒÖ¸¶¨µÄ±äÁ¿
Èë²Î£º±äÁ¿Ãû
·µ»Ø£º¶ÔÓ¦µÄ±äÁ¿¼ÇÂ¼
 */
static QSH_RECORD* Q_Sh_SearchVar(char* Name)
{
    QSH_RECORD* Index;
    for (Index = (QSH_RECORD*) &qShellVarTab$$Base; Index < (QSH_RECORD*) &qShellVarTab$$Limit; Index ++)
    {
        if (strcmp(Index->name, Name) == 0)
            return Index;
    }
    return (void *)0;
}
/* 
¹¦ÄÜ£ºÁĞ³öº¯Êı¶ÎÖĞµÄËùÓĞº¯Êı¼ÇÂ¼µÄÃèÊö×Ö·û´®
Èë²Î£ºÎŞ
·µ»Ø£ºÎŞ
 */
static void Q_Sh_ListFuns(void)
{
    
    QSH_RECORD* Index;
    for (Index = (QSH_RECORD*) &qShellFunTab$$Base; Index < (QSH_RECORD*) &qShellFunTab$$Limit; Index++)
    {	
        printf("%s\r\n",Index->desc);
    }
}



/* 
¹¦ÄÜ£ºlist help imformation
Èë²Î£ºÎŞ
·µ»Ø£ºÎŞ
 */
static void Q_Sh_HelpFuns(void)
{
    QSH_RECORD* Index;
    for (Index = (QSH_RECORD*) &qShellFunTab$$Base; Index < (QSH_RECORD*) &qShellFunTab$$Limit; Index++)
    {	
        printf("%s\r\n",Index->desc);
    }
}
/*
¹¦ÄÜ£ºÁĞ³ö±äÁ¿¶ÎÖĞµÄËùÓĞ±äÁ¿¼ÇÂ¼µÄÃèÊö×Ö·û´®
Èë²Î£ºÎŞ
·µ»Ø£ºÎŞ 
 */
static void Q_Sh_ListVars(void)
{
    QSH_RECORD* Index;
    for (Index = (QSH_RECORD*) &qShellVarTab$$Base; Index < (QSH_RECORD*) &qShellVarTab$$Limit; Index++)
    {
        printf("%s\r\n",Index->desc);
    }
}
/*
¹¦ÄÜ£º¼ÆËãmµÄn´Î·½
Èë²Î£ºm µ×Êı n ÃİÖ¸Êı
·µ»Ø£º¼ÆËã½á¹û 
 */
static unsigned int Q_Sh_Pow(unsigned int m,unsigned int n)
{
    unsigned int Result=1;	 
    while(n--)
        Result*=m;    
    return Result;
}
/*
¹¦ÄÜ£º½«×Ö·û×ª»»³ÉÊı×Ö£¬Ö§³Ö16½øÖÆ£¨´óĞ¡Ğ´×ÖÄ¸¾ù¿É£©£¬²»Ö§³Ö¸ºÊı
Èë²Î£ºStr Êı×Ö×Ö·û´® Res ½á¹û´æ·ÅµÄµØÖ·
·µ»Ø£º0 ³É¹¦ £¡0 Ê§°Ü
 */
static unsigned int Q_Sh_Str2Num(char*Str,unsigned int *Res)
{
    unsigned int Temp;
    unsigned int Num=0;			  
    unsigned int HexDec=10;
    char *p;
    p=Str;
    *Res=0;
    while(1)
    {
        if((*p>='A'&&*p<='F')||(*p=='X'))
            *p=*p+0x20;
        else if(*p=='\0')break;
        p++;
    }
    p=Str;
    while(1)
    {
        if((*p<='9'&&*p>='0')||(*p<='f'&&*p>='a')||(*p=='x'&&Num==1))
        {
            if(*p>='a')HexDec=16;	
            Num++;					
        }else if(*p=='\0')break;	
        else return 1;				
        p++; 
    } 
    p=Str;			    
    if(HexDec==16)		
    {
        if(Num<3)return 2;			
        if(*p=='0' && (*(p+1)=='x'))
        {
            p+=2;	
            Num-=2;
        }else return 3;
    }else if(Num==0)return 4;
    while(1)
    {
        if(Num)Num--;
        if(*p<='9'&&*p>='0')Temp=*p-'0';	
        else Temp=*p-'a'+10;				 
        *Res+=Temp*Q_Sh_Pow(HexDec,Num);		   
        p++;
        if(*p=='\0')break;
    }
    return 0;
}	
/*
¹¦ÄÜ£º´ÓÄ¸×Ö·û´®ÖĞ½ØµÃ×Ó×Ö·û´®
Èë²Î£ºBase Ä¸×Ö·û´®Ê×µØÖ· Start ´ı½Ø×Ó×Ö·ûµÄÆğÊ¼Î»ÖÃ End ´ı½Ø×Ö·û´®½áÊøÎ»ÖÃ
·µ»Ø£º×Ó×Ö·û´®Ê×µØÖ·
 */
static char *Q_Sh_StrCut(char *Base,unsigned int Start,unsigned int End)
{
    Base[End+1]=0;
    return (char *)((unsigned int)Base+Start);
}
/*
¹¦ÄÜ£º¶ÔÊäÈëµÄÃüÁî×Ö·û´®½øĞĞ´Ê·¨·ÖÎö
Èë²Î£º CmdStr ÃüÁî×Ö·û´®Ê×µØÖ· pQShLexResu ·ÖÎö½á¹û´æ·Å´¦
·µ»Ø£º0 ´Ê·¨·ÖÎöÊ§°Ü 1 ´Ê·¨·ÖÎö³É¹¦
 */
static unsigned int Q_Sh_LexicalAnalysis(char *CmdStr,Q_SH_LEX_RESU_DEF *pQShLexResu)
{
    unsigned int Index=0,TmpPos=0,ParamNum=0;
    
    while(1)
    {
        if( !( ( CmdStr[Index]>='a' && CmdStr[Index]<='z' ) || ( CmdStr[Index]>='A' && CmdStr[Index]<='Z' ) || ( CmdStr[Index]>='0' && CmdStr[Index]<='9' ) || CmdStr[Index]=='_' || CmdStr[Index]=='(') )
        {
            pQShLexResu->CallType=QSCT_ERROR;
            printf("Lexical Analysis Error: Function or Call name is illegal!!!\r\n");
            return 0;
        }
        if(CmdStr[Index]=='(')
        {
            if(Index==0)
            {
                pQShLexResu->CallType=QSCT_ERROR;
                printf("Lexical Analysis Error: Function or Call name is empty!!!\r\n");
                return 0;
            }
            pQShLexResu->CallEnd=(Index-1);
            TmpPos=++Index;
            break;
        }
        Index++;
    }
    
    while(1)
    {
        if( CmdStr[Index]<33||CmdStr[Index]>126 )
        {
            pQShLexResu->CallType=QSCT_ERROR;
            printf("Lexical Analysis Error: parameter include illegal character!!!\r\n");
            return 0;
        }
        if(CmdStr[Index]==',')
        {
            if(Index==TmpPos)
            {
                pQShLexResu->CallType=QSCT_ERROR;
                printf("Lexical Analysis Error: parameter include illegal comma!!!\r\n");
                return 0;
            }
            if(CmdStr[Index-1]=='\"')
            {
                if( ParamNum<QSH_FUN_PARA_MAX_NUM )
                {
                    pQShLexResu->ParaTypes[ParamNum]=QSFPT_STRING;
                    pQShLexResu->ParaStart[ParamNum]=TmpPos+1;
                    pQShLexResu->ParaEnd[ParamNum]=Index-2;
                    ParamNum++;
                }
                else
                {
                    ParamNum++;
                }
                TmpPos=Index+1;
            }
            else
            {
                if( ParamNum<QSH_FUN_PARA_MAX_NUM )
                {
                    pQShLexResu->ParaTypes[ParamNum]=QSFPT_NOSTRING;
                    pQShLexResu->ParaStart[ParamNum]=TmpPos;
                    pQShLexResu->ParaEnd[ParamNum]=Index-1;
                    ParamNum++;
                }
                else
                {
                    ParamNum++;
                }
                TmpPos=Index+1;
            }
        }
        else if(CmdStr[Index]==')')
        {
            if(Index==pQShLexResu->CallEnd+2)
            {
                pQShLexResu->ParaNum=0;
                break;
            }
            if(Index==TmpPos)
            {
                pQShLexResu->CallType=QSCT_ERROR;
                printf("Lexical Analysis Error: parameter include illegal comma!!!\r\n");
                return 0;
            }
            if(CmdStr[Index-1]=='\"')
            {
                if( ParamNum<QSH_FUN_PARA_MAX_NUM )
                {
                    pQShLexResu->ParaTypes[ParamNum]=QSFPT_STRING;
                    pQShLexResu->ParaStart[ParamNum]=TmpPos+1;
                    pQShLexResu->ParaEnd[ParamNum]=Index-2;
                    ParamNum++;
                }
                else
                {
                    ParamNum++;
                }
                pQShLexResu->ParaNum=ParamNum;
                break;
            }
            else
            {
                if( ParamNum<QSH_FUN_PARA_MAX_NUM )
                {
                    pQShLexResu->ParaTypes[ParamNum]=QSFPT_NOSTRING;
                    pQShLexResu->ParaStart[ParamNum]=TmpPos;
                    pQShLexResu->ParaEnd[ParamNum]=Index-1;
                    ParamNum++;
                }
                else
                {
                    ParamNum++;
                }
                pQShLexResu->ParaNum=ParamNum;
                break;
            }	
        }
        Index++;
    }
    pQShLexResu->CmdStr=CmdStr;
    pQShLexResu->CallStart=0;
    if( strcmp(Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->CallStart,pQShLexResu->CallEnd),"lf")==0 )
        pQShLexResu->CallType=QSCT_LIST_FUN;
    else if(strcmp(Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->CallStart,pQShLexResu->CallEnd),"lv")==0)
        pQShLexResu->CallType=QSCT_LIST_VAR;
    else if(strcmp(Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->CallStart,pQShLexResu->CallEnd),"get")==0)
        pQShLexResu->CallType=QSCT_READ_VAR;
    else if(strcmp(Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->CallStart,pQShLexResu->CallEnd),"set")==0)
        pQShLexResu->CallType=QSCT_WRITE_VAR;
    else if(strcmp(Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->CallStart,pQShLexResu->CallEnd),"read")==0)
        pQShLexResu->CallType=QSCT_READ_REG;
    else if(strcmp(Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->CallStart,pQShLexResu->CallEnd),"write")==0)
        pQShLexResu->CallType=QSCT_WRITE_REG;
    else if(strcmp(Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->CallStart,pQShLexResu->CallEnd),"help")==0)
        pQShLexResu->CallType=QSCT_HELP_FUN;
    else
        pQShLexResu->CallType=QSCT_CALL_FUN;
    return 1;
}
/*
¹¦ÄÜ£º¶Ô´Ê·¨·ÖÎöµÄ½á¹û½øĞĞÓï·¨·ÖÎö£¬²¢Ö´ĞĞ
Èë²Î£º pQShLexResu ´Ê·¨·ÖÎö½á¹û´æ·Å´¦
·µ»Ø£º0 Ê§°Ü 1 ³É¹¦
 */
static unsigned int Q_Sh_ParseAnalysis(Q_SH_LEX_RESU_DEF *pQShLexResu)
{
    QSH_RECORD *pRecord;
    unsigned int FunPara[QSH_FUN_PARA_MAX_NUM];
    unsigned int FunResu;
    unsigned int VarSet;
    unsigned int VarAddr;
    unsigned char i;
    unsigned char Ret=1;
    switch(pQShLexResu->CallType)
    {
    case QSCT_CALL_FUN:
    {
        pRecord=Q_Sh_SearchFun(Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->CallStart,pQShLexResu->CallEnd));
        if( pRecord == 0 )
        {
            printf("Parse Analysis Error: the input function have not been regist!!!\r\n");
            Ret=0;
            break;
        }			
        if(pQShLexResu->ParaNum>QSH_FUN_PARA_MAX_NUM)
        {	
            printf("Parse Analysis Error: function's parameter number can't over four!!!\r\n");
            Ret=0;
            break;
        }
        for(i=0;i<pQShLexResu->ParaNum;i++)
        {
            if(pQShLexResu->ParaTypes[i]==QSFPT_STRING)
            {
                FunPara[i]=(unsigned int)Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->ParaStart[i],pQShLexResu->ParaEnd[i]);					
            }
            else
            {
                if(Q_Sh_Str2Num(Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->ParaStart[i],pQShLexResu->ParaEnd[i]),&(FunPara[i]))!=0)
                {
                    printf("Parse Analysis Error: the input number string is illegal!!!\r\n");
                    Ret=0;
                    break;
                }
            }
        }
        FunResu=((QSH_FUNC_TYPE)(pRecord->addr))(FunPara[0],FunPara[1],FunPara[2],FunPara[3]);
        printf("return %d\r\n",FunResu);
    }
        break;
        
    case QSCT_READ_VAR:
    {
        pRecord=Q_Sh_SearchVar(Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->ParaStart[0],pQShLexResu->ParaEnd[0]));
        if( pRecord == 0 )
        {
            printf("Parse Analysis Error: the input variable have not been regist!!!\r\n");
            Ret=0;
            break;
        }
        if(pQShLexResu->ParaNum!=1)
        {	
            printf("Parse Analysis Error: this call must have only one parameter!!!\r\n");
            Ret=0;
            break;
        }
        if(strcmp(pRecord->typedesc,"u8")==0)
            printf("%s=%d\r\n",pRecord->name,*(unsigned char *)(pRecord->addr));
        else if(strcmp(pRecord->typedesc,"u16")==0)
            printf("%s=%d\r\n",pRecord->name,*(unsigned short *)(pRecord->addr));
        else if(strcmp(pRecord->typedesc,"u32")==0)
            printf("%s=%d\r\n",pRecord->name,*(unsigned int *)(pRecord->addr));
        else
        {
            printf("the describe of variable's type is illegal!!!\r\n");
            Ret=0;
            break;
        }
    }
        break;
        
    case QSCT_WRITE_VAR:
    {
        pRecord=Q_Sh_SearchVar(Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->ParaStart[0],pQShLexResu->ParaEnd[0]));
        if( pRecord == 0 )
        {
            printf("Parse Analysis Error: the input variable have not been regist!!!\r\n");
            Ret=0;
            break;
        }
        if(pQShLexResu->ParaNum!=2)
        {	
            printf("Parse Analysis Error: this call must have two parameter!!!\r\n");
            Ret=0;
            break;
        }
        if(pQShLexResu->ParaTypes[1]==QSFPT_STRING)
        {
            printf("Parse Analysis Error: this call's second parameter can't be string type!!!\r\n");
            Ret=0;
            break;
        }
        if(Q_Sh_Str2Num(Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->ParaStart[1],pQShLexResu->ParaEnd[1]),&VarSet)!=0)
        {
            printf("Parse Analysis Error: the input number string is illegal!!!\r\n");
            Ret=0;
            break;
        }
        if(strcmp(pRecord->typedesc,"u8")==0)
        {
            *(unsigned char *)(pRecord->addr)=VarSet;
            printf("%s=%d\r\n",pRecord->name,*(unsigned char *)(pRecord->addr));	
        }
        else if(strcmp(pRecord->typedesc,"u16")==0)
        {
            *(unsigned short *)(pRecord->addr)=VarSet;
            printf("%s=%d\r\n",pRecord->name,*(unsigned short *)(pRecord->addr));	
        }
        else if(strcmp(pRecord->typedesc,"u32")==0)
        {
            *(unsigned int *)(pRecord->addr)=VarSet;
            printf("%s=%d\r\n",pRecord->name,*(unsigned int *)(pRecord->addr));	
        }
        else
        {
            printf("the describe of variable's type is illegal!!!\r\n");
            Ret=0;
            break;
        }
    }
        break;
        
    case QSCT_READ_REG:
    {
        if(pQShLexResu->ParaNum!=1)
        {	
            printf("Parse Analysis Error: this call must have only one parameter!!!\r\n");
            Ret=0;
            break;
        }
        if(pQShLexResu->ParaTypes[0]==QSFPT_STRING)
        {
            printf("Parse Analysis Error: this call's parameter can not be string type!!!\r\n");
            Ret=0;
            break;
        }
        if(Q_Sh_Str2Num(Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->ParaStart[0],pQShLexResu->ParaEnd[0]),&VarAddr)!=0)
        {
            printf("Parse Analysis Error: the input number string is illegal!!!\r\n");
            Ret=0;
            break;
        }
        printf("*(0x%x)=0x%x\r\n",VarAddr,*(unsigned int *)VarAddr);
    }
        break;
        
    case QSCT_WRITE_REG:
    {
        if(pQShLexResu->ParaNum!=2)
        {	
            printf("Parse Analysis Error: this call must have two parameter!!!\r\n");
            Ret=0;
            break;
        }
        if(pQShLexResu->ParaTypes[0]==QSFPT_STRING)
        {
            printf("Parse Analysis Error: this call's first parameter can not be string type!!!\r\n");
            Ret=0;
            break;
        }
        if(pQShLexResu->ParaTypes[1]==QSFPT_STRING)
        {
            printf("Parse Analysis Error: this call's second parameter can not be string type!!!\r\n");
            Ret=0;
            break;
        }
        if(Q_Sh_Str2Num(Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->ParaStart[0],pQShLexResu->ParaEnd[0]),&VarAddr)!=0)
        {
            printf("Parse Analysis Error: the input number string is illegal!!!\r\n");
            Ret=0;
            break;
        }
        if(Q_Sh_Str2Num(Q_Sh_StrCut(pQShLexResu->CmdStr,pQShLexResu->ParaStart[1],pQShLexResu->ParaEnd[1]),&VarSet)!=0)
        {
            printf("Parse Analysis Error: the input number string is illegal!!!\r\n");
            Ret=0;
            break;
        }
        *(unsigned int *)VarAddr=VarSet;
        printf("*(0x%x)=0x%x\r\n",VarAddr,*(unsigned *)VarAddr);
    }
        break;
        
    case QSCT_LIST_FUN:
    {
        if(pQShLexResu->ParaNum>0)
        {	
            printf("Parse Analysis Error: this call must have no parameter!!!\r\n");
            Ret=0;
            break;
        }
        Q_Sh_ListFuns();
    }
        break;
    case QSCT_HELP_FUN:
    {
        if(pQShLexResu->ParaNum>0)
        {	
            printf("Parse Analysis Error: this call must have no parameter!!!\r\n");
            Ret=0;
            break;
        }
        Q_Sh_HelpFuns();
        
    }
        break;
        
    case QSCT_LIST_VAR:
    {
        if(pQShLexResu->ParaNum>0)
        {	
            printf("Parse Analysis Error: this call must have no parameter!!!\r\n");
            Ret=0;
            break;
        }
        Q_Sh_ListVars();
    }
        break;
        
    default:
    {	
        printf("Parse Analysis Error: can't get here!!!\r\n");
        Ret=0;
    }
    }
    return Ret;
}
/* 
shellÃÜÂë´æ·Å´¦£¬¿ª»úºóµÚÒ»´ÎÊ¹ÓÃshellĞèÒªÊäÈë´ËÃÜÂë
 */
static const char Q_ShellPassWord[]="123456";

/* 
¹¦ÄÜ£ºshell¶ÔÍâµÄ½Ó¿Ú£¬Ö´ĞĞÃüÁî
Èë²Î£ºIfCtrl Ö¸Ê¾ÊÇ·ñ½ÓÊÕµ½Ò»¸ö¿ØÖÆ×Ö·û CmdStr ´Ó´®¿ÚµÃµ½µÄÃüÁî×Ö·û´®
·µ»Ø£º1 ³É¹¦ 0 Ê§°Ü
 */
unsigned int Q_Sh_CmdHandler(unsigned int IfCtrl,char *CmdStr)
{
    Q_SH_LEX_RESU_DEF Q_Sh_LexResu;
    static unsigned char IfWaitInput=0;
    if(shell.ifpass)
    {
        if(IfCtrl==1)
        {
            if(((unsigned short *)CmdStr)[0]==0x445b)
            {}
            else if(((unsigned short *)CmdStr)[0]==0x435b)
            {}
            else
                printf("CtrlCode:%x\n\r",((unsigned short *)CmdStr)[0]);
            return 0;
        }
        if( Q_Sh_LexicalAnalysis(CmdStr,&Q_Sh_LexResu) == 0 )//´Ê·¨·ÖÎö
            return 0;
        if( Q_Sh_ParseAnalysis(&Q_Sh_LexResu) == 0 )//Óï·¨·ÖÎö
            return 0;	 
        return 1;
    }
    else
    {
        if(IfWaitInput==0)
        {
            printf("Please input shell password:");
            IfWaitInput=1;
        }
        else
        {
            if(strcmp(CmdStr,Q_ShellPassWord)==0)
            {
                shell.ifpass=1;
                printf("password right!\r\n");
            }
            else
            {
                printf("password wrong!\r\n");
                printf("Please input shell password again:");
            }
        }
        return 0;
    }
}

#else
/*
µ±USE_Q_SHELL_FUNCTION=0¼´shell¹¦ÄÜ±»¹Ø±Õºó£¬ÎªÁË²»ĞèÒªĞŞ¸ÄÔ­³ÌĞò¶ø¼ÓÈë
 */
unsigned int Q_Sh_CmdHandler(unsigned int IfCtrl,char *CmdStr)
{
    return 0;
}

#endif




/*
             * handle control key
             * up key  : 0x1b 0x5b 0x41
             * down key: 0x1b 0x5b 0x42
             * right key:0x1b 0x5b 0x43
             * left key: 0x1b 0x5b 0x44
 */


os_err_t  Q_Sh_rx_callback(char data)
{
    char Recieve=data;
    // osmemset(&shell,0,sizeof(struct finsh_shell));
    if (Recieve == 0x1b)
    {
        shell.status = WAIT_SPEC_KEY;
        goto end;
    }
    else if (shell.status == WAIT_SPEC_KEY)
    {
        if (Recieve == 0x5b)
        {
            shell.status = WAIT_FUNC_KEY;
            goto end;
        }
        
        shell.status = WAIT_NORMAL;
    }
    else if (shell.status == WAIT_FUNC_KEY)
    {
        shell.status = WAIT_NORMAL;
        
        if (Recieve == 0x44) /* left key */
        {
            if (shell.line_curpos)
            {
                printf("\b");
                shell.line_curpos --;
            }
            
            goto end;
        }
        else if (Recieve == 0x43) /* right key */
        {
            if (shell.line_curpos < shell.line_position)
            {
                printf("%c", shell.line[shell.line_curpos]);
                shell.line_curpos ++;
            }
            goto end;
            
        }
        
    }
    
    
    if(Recieve==13)//»Ø³µ
    {
        
        shell.line[shell.line_position]=0;
        printf("\n\r");	
        if(shell.line_position>0||!(shell.ifpass))
        {	
            Q_Sh_CmdHandler(0,shell.line);
        }
        shell.line_position=0;
        shell.line_curpos=0;
        printf("\n\rWSHOS>>");
    }
    /* handle backspace key */
    else if(Recieve == 0x7f || Recieve == 0x08)//»ØÉ¾
    {
        if(shell.line_curpos>0)
        {
            shell.line_position--;
            shell.line_curpos--;
            if (shell.line_position > shell.line_curpos)
            {
                u16 i;
                os_memmove(&shell.line[shell.line_curpos],
                        &shell.line[shell.line_curpos + 1],
                        shell.line_position - shell.line_curpos);
                shell.line[shell.line_position] = 0;
                
                printf("\b%s  \b", &shell.line[shell.line_curpos]);
                
                /* move the cursor to the origin position */
                for (i = shell.line_curpos; i <= shell.line_position; i++)
                    printf("\b");
            }else
            {
                shell.line[(shell.line_position)]=0;
                printf("\b \b");	
            }
            
        }
    }
    if(shell.line_position>=SHELL_CMD_SIZE)
    {				
        shell.line_position=0;//³¬³ö·¶Î§£¬¾ÍÍ£Ö¹¼ÓÈëĞÂµÄÁË¡£
        shell.line_curpos=0;//³¬³ö·¶Î§£¬¾ÍÍ£Ö¹¼ÓÈëĞÂµÄÁË¡
    }
    if(Recieve>=0x20)
    {
        
        
        if (shell.line_position > shell.line_curpos)
        {
            u16 i;
            
            os_memmove(&shell.line[shell.line_curpos + 1],
                    &shell.line[shell.line_curpos],
                    shell.line_position - shell.line_curpos);
            shell.line[shell.line_curpos] = Recieve;
            shell.line[shell.line_position+1] = 0;
            printf("%s", &shell.line[shell.line_curpos]);
            
            /* move the cursor to the origin position */
            for (i = shell.line_curpos; i < shell.line_position; i++)
                printf("\b");
        }else
        {
            shell.line[(shell.line_position)]=Recieve;
            shell.line[shell.line_curpos+1] = 0;						
            printf("%c",Recieve);			
        }
        
        shell.line_position++;
        shell.line_curpos++;
    }
    
    
    
    
end:   
    return SUCCESS;
}

