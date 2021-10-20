//zyk
#ifndef _SINGLE_NEURON_PID_H_
#define _SINGLE_NEURON_PID_H_

#include<iostream>
#include<math.h>

// class PID_SingleNeuron
// {
// private:
//     float Prate;                  /*比例学习速度*/
//     float Irate;                  /*积分学习速度*/
//     float Drate;                  /*微分学习速度*/
// 
// 	float Kout;                  /*神经元输出比例*/
// 	  
//     float e_pre_1;
//     float e_pre_2;
// 
//     float Ref_r;
// 	float Act_y;
//     float u_1;     
// public:
// 	float wP_1;                     /*上一次的比例加权系数*/
//     float wI_1;                     /*上一次的积分加权系数*/
//     float wD_1;                     /*上一次的微分加权系数*/ 
// 	float e;
// 
//     PID_SingleNeuron();
// 	~PID_SingleNeuron(){};
//     PID_SingleNeuron(float pr,float ir,float dr,float pw, float iw,float dw,float ko);
//     float NeuronPid_control(float tar,float act);	
//     void NeuronPid_show();
// };

class PID_SingleNeuron
{
private:
    float Prate;                  /*比例学习速度*/
    float Irate;                  /*积分学习速度*/
    float Drate;                  /*微分学习速度*/

	float Kout;                  /*神经元输出比例*/
	  
    float e_pre_1;
    float e_pre_2;

    float Ref_r;
	float Act_y;
    float u_1;     
public:
	float wP_1;                     /*上一次的比例加权系数*/
    float wI_1;                     /*上一次的积分加权系数*/
    float wD_1;                     /*上一次的微分加权系数*/ 
	float e;

	~PID_SingleNeuron(){};
    PID_SingleNeuron(float pr,float ir,float dr,float pw, float iw,float dw,float ko);
    float NeuronPid_control(float tar,float act);	
};

// ////位置式PID
// class PID_position
// {
// private:
//     float kp;//比例系数
//     float ki;//积分系数
//     float kd;//微分系数
//     float target;//目标值
//     float actual;//实际值
//     float e;//误差
//     float e_pre;//上一次误差
//     float integral;//积分项
// public:
//     PID_position();
//     ~PID_position(){};
//     PID_position(float p,float i,float d);
//     float pid_control(float tar,float act);//执行PID控制
//     void pid_show();//显示PID控制器的内部参数
// };

// ///增量式PID
// class PID_incremental
// {
// private:
//     float kp;
//     float ki;
//     float kd;
//     float target;
//     float actual;
//     float e;
//     float e_pre_1;
//     float e_pre_2;
//     float A;
//     float B;
//     float C;
// public:
//     PID_incremental();
//     PID_incremental(float p,float i,float d);
//     float pid_control(float tar,float act);
//     void pid_show();
// };


#endif    