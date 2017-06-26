#include<iostream>
#include"fuzzy_controller.h"
#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3


int main()
{
	float target=60;
	float actual=0;
	float u=0;
	int ruleMatrix[7][7]={{NB,NB,NM,NM,NS,ZO,ZO},
	                      {NB,NB,NM,NS,NS,ZO,PS},
						  {NM,NM,NM,NS,ZO,PS,PS},
	                      {NM,NM,NS,ZO,PS,PM,PM},
	                      {NS,NS,ZO,PS,PS,PM,PM},
	                      {NS,ZO,PS,PM,PM,PM,PB},
	                      {ZO,ZO,PM,PM,PM,PB,PB}};
	float e_mf_paras[21]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
	float de_mf_paras[21]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
	float u_mf_paras[21]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
    Fuzzy_controller fuzzy(100,65,50);
	fuzzy.setMf("trimf",e_mf_paras,"trimf",de_mf_paras,"trimf",u_mf_paras);
	fuzzy.setRule(ruleMatrix);
	cout<<"num   target    actual"<<endl;
	/*fuzzy.showInfo();*/
	for(int i=0;i<100;i++)
	{
		u=fuzzy.realize(target,actual);
		actual+=u;
		cout<<i<<"  "<<target<<"   "<<actual<<endl;
	}
	fuzzy.showInfo();
	system("pause");
	return 0;
}