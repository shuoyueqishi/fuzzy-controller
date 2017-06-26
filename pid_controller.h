#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_
//增量式PID
class PID_incremental
{
private:
	float kp;
	float ki;
	float kd;
	float target;
	float actual;
	float e;
	float e_pre_1;
	float e_pre_2;
	float A;
	float B;
	float C;
public:
	PID_incremental();
	PID_incremental(float p,float i,float d);
	float pid_control(float tar,float act);
	void pid_show();
};

//位置式PID
class PID_position
{
private:
	float kp;
	float ki;
	float kd;
	float target;
	float actual;
	float e;
	float e_pre;
	float integral;
public:
	PID_position();
	PID_position(float p,float i,float d);
	float pid_control(float tar,float act);
	void pid_show();
};

#endif