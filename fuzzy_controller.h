#ifndef FUZZY_CONTROLLER_H_
#define FUZZY_CONTROLLER_H_
#include<iostream>
#include<string>
using namespace std;

class Fuzzy_controller
{
public:
	const static int N=7;//定义量化论域模糊子集的个数
private:
	float target;//系统的控制目标
	float actual;//采样获得的实际值
	float e;     //误差
	float e_pre; //上一次的误差
	float de;    //误差的变化率
	float emax;  //误差基本论域上限
	float demax; //误差辩化率基本论域的上限
	float umax;  //输出的上限
	float Ke;    //Ke=n/emax,量化论域为[-3,-2,-1,0,1,2,3]
	float Kde;   //Ke=n/demax,量化论域为[-3,-2,-1,0,1,2,3]
	float Ku;    //Ke=umax/n,量化论域为[-3,-2,-1,0,1,2,3]
	int rule[N][N];//模糊规则表
	string mf_t_e;   //e的隶属度函数类型
	string mf_t_de;  //de的隶属度函数类型
	string mf_t_u;   //u的隶属度函数类型
	float *e_mf_paras; //误差的隶属度函数的参数
	float *de_mf_paras;//误差的偏差隶属度函数的参数
	float *u_mf_paras; //输出的隶属度函数的参数

public:
	Fuzzy_controller(float e_max,float de_max,float u_max);
	~Fuzzy_controller();
	float trimf(float x,float a,float b,float c);          //三角隶属度函数
	float gaussmf(float x,float ave,float sigma);          //正态隶属度函数
	float trapmf(float x,float a,float b,float c,float d); //梯形隶属度函数
	//设置模糊隶属度函数的参数
	void setMf(const string & mf_type_e,float *e_mf,const string & mf_type_de,float *de_mf,const string & mf_type_u,float *u_mf);
	void setRule(int rulelist[N][N]);                          //设置模糊规则
	float realize(float t,float a);              //实现模糊控制
	void showInfo();                                      //显示该模糊控制器的信息
	void showMf(const string & type,float *mf_paras);      //显示隶属度函数的信息
};

#endif
