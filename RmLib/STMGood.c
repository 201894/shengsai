/** @file STMGood.c
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief data analysis and send message to the motor, used in pid debug mainly
 *
 */
#include "STMGood.h"
#include "stm32f4xx_hal.h"
#include "bsp_uart.h"
#include "main_task.h"
 //-----------------------------------ͷ�ļ�����
 
//-----------------------------------���������������±������ܸı䣩
char Count=0;
char Cmd1[20];
char Cmd2[80];
char Data[100];
char Str1[100];
double xx[100];
//-----------------------------------���������������ϱ������ܸı䣩

float _kp,_ki,_kd; 
float _kkp,_kki,_kkd;
float target,maxout1,maxout2;
//-------------------------------------------------------------------------------
int cmd(char *Cmd,int n)
{
  //���ɵĴ�����д�ڴ˺�����
  
  if(CompStr(Cmd,"a"))
  {

    return 0;
  }
  if(CompStr(Cmd,"b"))
  {
		
    return 0;
  }
  if(CompStr(Cmd,"c"))
  {
    return 0;
  }
  if(CompStr(Cmd,"d"))
  {
    return 0;
  }
	
	if(CompStr(Cmd,"u"))
  {
      _kp = _kp + 0.05;
//	 printf("P=%f  I=%f  D=%f\r\n",_kp,_ki,_kd);
    return 0;
  }
	if(CompStr(Cmd,"i"))
  {
   _kki = _kki + 0.05;
  //  printf("P=%f  I=%f  D=%f\r\n",_kp,_kki,_kd);
    return 0;
  }
	if(CompStr(Cmd,"o"))
  {
    _kkp = _kkp + 0.05;
 //   printf("P=%f  I=%f  D=%f\r\n",_kp,_ki,_kd);
    return 0;
  }
	if(CompStr(Cmd,"j"))
  {
        _kp = _kp - 0.05;

    return 0;
  }
	if(CompStr(Cmd,"k"))
  {
     _kki = _kki - 0.05;
    return 0;
  }
	if(CompStr(Cmd,"l"))
  {
      _kkp = _kkp - 0.05;
    return 0;
  }
	if(CompStr(Cmd,"p"))
  {
    _kp = 0.0;
		_kkp = 0.0;
		_kki = 0.0;
//		printf("P=%f  I=%f  D=%f\r\n",P,I,D);
    return 0;
  }
	
	return 0;
}
void multi1(int n)
{
	_kp= xx[1];
	_ki = xx[2];
    _kd = xx[3];
	printf("################\r\n");
	printf("PO:%f IO:%f DO:%f\r\n",_kp,_ki,_kd);
}

void multi2(int n)
{
	_kkp = xx[1];
	_kki  = xx[2];
//	_kkd = xx[3];
	printf("################\r\n");
	printf("PI:%f II:%f DI:%f\r\n",_kkp,_kki,_kkd);		
}
void multi3(int n)
{
   gimbal_yaw.target  = xx[1];
   logic_data.launch_flag =(int)xx[2];
   printf("################\r\n");
   printf("maxout1:%f maxout2:%f\r\n",maxout1,maxout2);
}
void multi4(int n)
{
     fric_data.speed = xx[1];
     _kkd = (int)xx[2];	
//	gimbal.target[0]  = xx[2];
}

void multi5(int n)
{

}
void multi6(int n)
{

}
void multi7(int n)
{

}
void multi8(int n)
{

}
void multi9(int n)
{

}
void part1(int xx)
{

}
void part2(int xx)
{
  
}
void part3(int xx)
{
  
}
void part4(int xx)
{
  
}
void part5(int xx)
{
  
}
void part6(int xx)
{
  
}

void go()
{
	
}
void back()
{
  
}
void left()
{
  
}
void right()
{
  
}
void stopcar()
{
	
}
void goleft()
{
  
}
void goright()
{
  
}
void backleft()
{
  
}
void backright()
{
  
}


void go1()
{
  printf("Go1\r\n");
}
void back1()
{
  
}
void left1()
{
  
}
void right1()
{
  
}
void goleft1()
{
  
}
void goright1()
{
  
}
void backleft1()
{
  
}
void backright1()
{
  
}

//-----------------------------------------------------------------------



int Command(char *Cmd,int n)
{
//------------------------------����Ϊ�������-----------------------------
	if(CompStr(Cmd,"*1"))
	{
		multi1(n);
		return 0;
	}
	if(CompStr(Cmd,"*2"))
	{
		multi2(n);
		return 0;
	}
	if(CompStr(Cmd,"*3"))
	{
		multi3(n);
		return 0;
	}
	if(CompStr(Cmd,"*4"))
	{
		multi4(n);
		return 0;
	}
	if(CompStr(Cmd,"*5"))
	{
		multi5(n);
		return 0;
	}
	if(CompStr(Cmd,"*6"))
	{
		multi6(n);
		return 0;
	}
	if(CompStr(Cmd,"*7"))
	{
		multi7(n);
		return 0;
	}
	if(CompStr(Cmd,"*8"))
	{
		multi8(n);
		return 0;
	}
	if(CompStr(Cmd,"*9"))
	{
		multi9(n);
		return 0;
	}
//------------------------------����Ϊ�������ƣ�����Ϊxx[1]-------------------
	if(CompStr(Cmd,"#1"))
	{
		part1(xx[1]);
		return 0;
	}
	if(CompStr(Cmd,"#2"))
	{
		part2(xx[1]);
		return 0;
	}
	if(CompStr(Cmd,"#3"))
	{
		part3(xx[1]);
		return 0;
	}
	if(CompStr(Cmd,"#4"))
	{
		part4(xx[1]);
		return 0;
	}
	if(CompStr(Cmd,"#5"))
	{
		part5(xx[1]);
		return 0;
	}
	if(CompStr(Cmd,"#6"))
	{
		part6(xx[1]);
		return 0;
	}
//------------------------------����ΪС������-----------------------------
	if(CompStr(Cmd,"@1"))
	{
		//ǰ��
		go();
		return 0;
	}
	if(CompStr(Cmd,"@2"))
	{
		//����
		back();
		return 0;
	}
	if(CompStr(Cmd,"@3"))
	{
		//��ת
		left();
		return 0;
	}
	if(CompStr(Cmd,"@4"))
	{
		//��ת
		right();
		return 0;
	}
	if(CompStr(Cmd,"@5"))
	{
		//ֹͣ
		stopcar();
		return 0;
	}
	if(CompStr(Cmd,"@6"))
	{
		//��ǰ
		goleft();
		return 0;
	}
	if(CompStr(Cmd,"@7"))
	{
		//��ǰ
		goright();
		return 0;
	}
	if(CompStr(Cmd,"@8"))
	{
		//���
		backleft();
		return 0;
	}
	if(CompStr(Cmd,"@9"))
	{
		//�Һ�
		backright();
		return 0;
	}
	
	
	if(CompStr(Cmd,"$1"))
	{
		//ǰ��(���¹��ܼ�)
		go1();
		return 0;
	}
	if(CompStr(Cmd,"$2"))
	{
		//����(���¹��ܼ�)
		back1();
		return 0;
	}
	if(CompStr(Cmd,"$3"))
	{
		//��ת(���¹��ܼ�)
		left1();
		return 0;
	}
	if(CompStr(Cmd,"$4"))
	{
		//��ת(���¹��ܼ�)
		right1();
		return 0;
	}
	if(CompStr(Cmd,"$6"))
	{
		//��ǰ(���¹��ܼ�)
		goleft1();
		return 0;
	}
	if(CompStr(Cmd,"$7"))
	{
		//��ǰ(���¹��ܼ�)
		goright1();
		return 0;
	}
	if(CompStr(Cmd,"$8"))
	{
		//���(���¹��ܼ�)
		backleft1();
		return 0;
	}
	if(CompStr(Cmd,"$9"))
	{
		//�Һ�(���¹��ܼ�)
		backright1();
		return 0;
	}	
	cmd(Cmd,n);
  return 0;
  //------------------------------------------
}


//--------------------------------------------����Ϊ�Զ�������������ı�-------------------------

void Dealdata(int Rx)                  //������λ�����͹���������
{
	char i;
	if (Rx=='(' && Count==0)
	{
		Data[Count++]=Rx;
  }
	else if (Count>0)
	{
		Data[Count++]=Rx;
  }
	if (Rx==')')
	{
	  Data[Count]='\0';
	  Count=0;
	  for (i=0;i<Strlen(Data)-1;i++)
	  {
		  	Data[i]=Data[i+1];
    }
	  Data[Strlen(Data)-2]='\0';
		SplitStr(Data,Cmd1,Cmd2);
		Command(Cmd1,DealStr(Cmd2));
		for(i=0;i<100;i++)
		{
			Cmd2[i]=0;
    }
  }
}

//-----------------------------------------------

int DealStr(char *Str)
{
	int i;
	int m=0;
	int n=1;
	int len=Strlen(Str);
	if(len==0)
	{
		return 0;
  }
	for(i=0;i<len;i++)
	{
		if (CompStr(SubStr(Str,i,i+1)," "))
		{
			xx[n]=StrToFloat(SubStr(Str,m,i));
			n++;
			m=i+1;
    }
		else
		{
			if(len-i==1)
			{
				xx[n]=StrToFloat(SubStr(Str,m,len));
      }
			continue;
    }
  }
	return n;
}

char *SubStr(char *Str,int start,int final)            //��ȡ���ַ���
{
	int i;
	for(i=start;i<final;i++)
	{
		*(Str1+i-start)=*(Str+i);
  }
	*(Str1+final-start)='\0';
	return Str1;
}

void SplitStr(char *Str,char *Str1,char *Str2)           //�ָ��ַ���
{
	int i;
	int k=FirstSpace(Str);
	int len=Strlen(Str);
	for (i=0;i<k;i++)
	{
		Str1[i]=Str[i];
  }
	Str1[k]='\0';
	for (i=k+1;i<len;i++)
	{
		Str2[i-k-1]=Str[i];
  }
	Str2[len]='\0';
	if(k==-1)
	{
		CopyStr(Str,Str1);
		Str2[0]='\0';
  }
}

int FirstSpace(char *Str)                             //Ѱ���ַ����е�һ���ո�λ��
{
	int i;
	for (i=0;i<100;i++)
	{
		if (Str[i]==' ')
		{
			return i;
    }
		else if(Str[i]=='\0')
		{
			return -1;
    }
			
  }
	return -1;
}

int Strlen(char *Str)                                 //��ȡ�ַ�������
{
	 int len=0;
	 while(*(Str+len++)!='\0');
	 return --len;
}

int CompStr(char *Str1,char *Str2)                     //�Ƚ������ַ����Ƿ���ͬ
{
	int i;
	int aStr=Strlen(Str1);
	int bStr=Strlen(Str2);
	if(aStr==bStr)
	{
		for (i=0;i<aStr;i++)
		{
			if(*(Str1+i)==*(Str2+i))
				continue;
			else
				return 0;
    }
		return 1;
  }
	else
	{
		return 0;
  }
}

int CopyStr(char *Str1,char *Str2)                      //�����ַ�����Դ�ַ�����Str1��Ŀ�꣺Str2
{
	int i;
	for (i=0;i<100;i++)
	{
		if(Str1[i]!='\0')
		{
			Str2[i]=Str1[i];
    }
		else
		{
			Str2[i]=Str1[i];
			return 1;
    }
  }
	return 1;
}

int StrToInt(char *Str)                                 //�ַ���ת��Ϊ����
{
	return (int)StrToFloat(Str);
}

float StrToFloat(char *Str)                              //�ַ���ת��ΪС��
{
     char i,j,k,negative=0;
    #define s_temp Str
    double result=0,result_1=0;
    for(i=0;i<10;i++)
    {
       j=Str[i];
       if(j==0||((j<'0'||j>'9')&&(j!='.')&&(j!='-')))
			 break;             
    } 
    k=j=i;
    for(i=0;i<j;i++)
    {
        if(s_temp[i]=='.')
				break;         
    }
    
    for(j=0;j<i;j++)
    {
        if(s_temp[j]=='-')
        {
		  		 negative=1;
           continue;
        }        
        result=result*10+(s_temp[j]-'0');            
    }
    j++;
    i=j;
    for(;j<k;j++)
    {
        if(s_temp[j]<'0'||s_temp[j]>'9')
			 	break;
        result_1=result_1*10+(s_temp[j]-'0');      
    }
    for(j=0;j<(k-i);j++)
		result_1*=0.1;
    result+=result_1;
    
    if(negative)result=-result;
    return result;
}
//------------------------------------����������С������
void senddouble1(double x)
{
	printf(",A:");
	printf("%.2f",x);
}
void senddouble2(double x)
{
	printf(",B:");
	printf("%f",x);
}
void senddouble3(double x)
{
	printf(",C:");
	printf("%f",x);
}
void senddouble4(double x)
{
	printf(",D:");
	printf("%f",x);
}
void senddouble5(double x)
{
	printf(",E:");
	printf("%f",x);
}
void senddouble6(double x)
{
	printf(",F:");
	printf("%f",x);
}
void senddouble7(double x)
{
	printf(",G:");
	printf("%f",x);
}
void senddouble8(double x)
{
	printf(",H:");
	printf("%f",x);
}
void senddouble9(double x)
{
	printf(",I:");
	printf("%f",x);
}
void senddouble10(double x)
{
	printf(",J:");
	printf("%f",x);
}
//---------------------------������������������
void sendint1(int x)
{
	printf(",A:");
	printf("%d",x);
}
void sendint2(int x)
{
	printf(",B:");
	printf("%d",x);
}
void sendint3(int x)
{
	printf(",C:");
	printf("%d",x);
}
void sendint4(int x)
{
	printf(",D:");
	printf("%d",x);
}
void sendint5(int x)
{
	printf(",E:");
	printf("%d",x);
}
void sendint6(int x)
{
	printf(",F:");
	printf("%d",x);
}
void sendint7(int x)
{
	printf(",G:");
	printf("%d",x);
}
void sendint8(int x)
{
	printf(",H:");
	printf("%d",x);
}
void sendint9(int x)
{
	printf(",I:");
	printf("%d",x);
}
void sendint10(int x)
{
	printf(",J:");
	printf("%d",x);
}
