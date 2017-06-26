#include"fuzzy_controller.h"


Fuzzy_controller::Fuzzy_controller(float e_max,float de_max,float u_max):
target(0),actual(0),emax(e_max),demax(de_max),umax(u_max),e_mf_paras(NULL),de_mf_paras(NULL),u_mf_paras(NULL)
{
   e=target-actual;
   e_pre=0;
   de=e-e_pre;
   Ke=(N/2)/emax;
   Kde=(N/2)/demax;
   Ku=umax/(N/2);
   mf_t_e="trimf";
   mf_t_de="trimf";
   mf_t_u="trimf";
}
//三角隶属度函数
float Fuzzy_controller::trimf(float x,float a,float b,float c)
{
   float u;
   if(x>=a&&x<=b)
	   u=(x-a)/(b-a);
   else if(x>b&&x<=c)
	   u=(c-x)/(c-b);
   else
	   u=0.0;
   return u;

}
//正态隶属度函数
float Fuzzy_controller::gaussmf(float x,float ave,float sigma) 
{
	float u;
	if(sigma<0)
	{
	   cout<<"In gaussmf, sigma must larger than 0"<<endl;
	}
	u=exp(-pow(((x-ave)/sigma),2));
	return u;
}
//梯形隶属度函数
float Fuzzy_controller::trapmf(float x,float a,float b,float c,float d)
{
    float u;
	if(x>=a&&x<b)
		u=(x-a)/(b-a);
	else if(x>=b&&x<c)
        u=1;
	else if(x>=c&&x<=d)
		u=(d-x)/(d-c);
	else
		u=0;
	return u;
}
//设置模糊规则
void Fuzzy_controller::setRule(int rulelist[N][N])
{
	for(int i=0;i<N;i++)
	   for(int j=0;j<N;j++)
	     rule[i][j]=rulelist[i][j];
}

//设置模糊隶属度函数的类型和参数
void Fuzzy_controller::setMf(const string & mf_type_e,float *e_mf,const string & mf_type_de,float *de_mf,const string & mf_type_u,float *u_mf)
{
	if(mf_type_e=="trimf"||mf_type_e=="gaussmf"||mf_type_e=="trapmf")
	    mf_t_e=mf_type_e;
	else
		cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;

	if(mf_type_de=="trimf"||mf_type_de=="gaussmf"||mf_type_de=="trapmf")
	    mf_t_de=mf_type_de;
	else
		cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;

	if(mf_type_u=="trimf"||mf_type_u=="gaussmf"||mf_type_u=="trapmf")
	    mf_t_u=mf_type_u;
	else
		cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;
	e_mf_paras=new float [N*3];
	de_mf_paras=new float [N*3];
	u_mf_paras=new float [N*3];
    for(int i=0;i<N*3;i++)
	   e_mf_paras[i]=e_mf[i];
	for(int i=0;i<N*3;i++)
	   de_mf_paras[i]=de_mf[i];
	for(int i=0;i<N*3;i++)
	   u_mf_paras[i]=u_mf[i];
}
//实现模糊控制
float Fuzzy_controller::realize(float t,float a)   
{
	float u_e[N],u_de[N],u_u[N];
	int u_e_index[3],u_de_index[3];//假设一个输入最多激活3个模糊子集
	float u;
	int M;
	target=t;
	actual=a;
    e=target-actual;
	de=e-e_pre;
	e=Ke*e;
	de=Kde*de;
	if(mf_t_e=="trimf")
		M=3;               //三角函数有三个参数
    else if(mf_t_e=="gaussmf")
	    M=2;              //正态函数有两个参数
	else if(mf_t_e=="trapmf")
		M=4;              //梯形函数有四个参数
	int j=0;
	for(int i=0;i<N;i++)
	{
		u_e[i]=trimf(e,e_mf_paras[i*M],e_mf_paras[i*M+1],e_mf_paras[i*M+2]);//e模糊化，计算它的隶属度
		if(u_e[i]!=0)
            u_e_index[j++]=i;                                              //存储被激活的模糊子集的下标，可以减小计算量
	}
	for(;j<3;j++)u_e_index[j]=0;

	if(mf_t_e=="trimf")
		M=3;              //三角函数有三个参数
    else if(mf_t_e=="gaussmf")
	    M=2;              //正态函数有两个参数
	else if(mf_t_e=="trapmf")
		M=4;               //梯形函数有四个参数
	j=0;
	for(int i=0;i<N;i++)
	{
		u_de[i]=trimf(de,de_mf_paras[i*M],de_mf_paras[i*M+1],de_mf_paras[i*M+2]);//de模糊化，计算它的隶属度
		if(u_de[i]!=0)
			u_de_index[j++]=i;                                                    //存储被激活的模糊子集的下标，可以减小计算量
	}
	for(;j<3;j++)u_de_index[j]=0;

	float den=0,num=0;
	for(int m=0;m<3;m++)
		for(int n=0;n<3;n++)
		{
		   num+=u_e[u_e_index[m]]*u_de[u_de_index[n]]*rule[u_e_index[m]][u_de_index[n]];
		   den+=u_e[u_e_index[m]]*u_de[u_de_index[n]];
		}
	u=num/den;
	u=Ku*u;
	if(u>=umax)   u=umax;
	else if(u<=-umax)  u=-umax;
	e_pre=e;
	return u;
}
void Fuzzy_controller::showMf(const string & type,float *mf_paras)
{
    int tab;
	if(type=="trimf")
		tab=2;
	else if(type=="gaussmf")
		tab==1;
	else if(type=="trapmf")
		tab=3;
	cout<<"函数类型："<<mf_t_e<<endl;
	cout<<"函数参数列表："<<endl;
	float *p=mf_paras;
	for(int i=0;i<N*(tab+1);i++)
	  {
		  cout.width(3);
	      cout<<p[i]<<"  ";
		  if(i%3==tab)
			  cout<<endl;
	  }
}
void Fuzzy_controller::showInfo()
{
   cout<<"Info of this fuzzy controller is as following:"<<endl;
   cout<<"基本论域e：["<<-emax<<","<<emax<<"]"<<endl;
   cout<<"基本论域de：["<<-demax<<","<<demax<<"]"<<endl;
   cout<<"基本论域u：["<<-umax<<","<<umax<<"]"<<endl;
   cout<<"误差e的模糊隶属度函数参数："<<endl;
   showMf(mf_t_e,e_mf_paras);
   cout<<"误差变化率de的模糊隶属度函数参数："<<endl;
   showMf(mf_t_de,de_mf_paras);
   cout<<"输出u的模糊隶属度函数参数："<<endl;
   showMf(mf_t_u,u_mf_paras);
   cout<<"模糊规则表："<<endl;
   for(int i=0;i<N;i++)
   {
	 for(int j=0;j<N;j++)
	   {
		 cout.width(3);
		 cout<<rule[i][j]<<"  ";
	    }
	   cout<<endl;
   }
   cout<<endl;
   cout<<"误差的量化比例因子Ke="<<Ke<<endl;
   cout<<"误差变化率的量化比例因子Kde="<<Kde<<endl;
   cout<<"输出的量化比例因子Ku="<<Ku<<endl;
   cout<<"设定目标target="<<target<<endl;
   cout<<"误差e="<<e<<endl;
   cout<<endl;
}