//zyk
#ifndef _SINGLE_NEURON_PID_H_
#define _SINGLE_NEURON_PID_H_

#include<iostream>
#include<math.h>

// class PID_SingleNeuron
// {
// private:
//     float Prate;                  /*����ѧϰ�ٶ�*/
//     float Irate;                  /*����ѧϰ�ٶ�*/
//     float Drate;                  /*΢��ѧϰ�ٶ�*/
// 
// 	float Kout;                  /*��Ԫ�������*/
// 	  
//     float e_pre_1;
//     float e_pre_2;
// 
//     float Ref_r;
// 	float Act_y;
//     float u_1;     
// public:
// 	float wP_1;                     /*��һ�εı�����Ȩϵ��*/
//     float wI_1;                     /*��һ�εĻ��ּ�Ȩϵ��*/
//     float wD_1;                     /*��һ�ε�΢�ּ�Ȩϵ��*/ 
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
    float Prate;                  /*����ѧϰ�ٶ�*/
    float Irate;                  /*����ѧϰ�ٶ�*/
    float Drate;                  /*΢��ѧϰ�ٶ�*/

	float Kout;                  /*��Ԫ�������*/
	  
    float e_pre_1;
    float e_pre_2;

    float Ref_r;
	float Act_y;
    float u_1;     
public:
	float wP_1;                     /*��һ�εı�����Ȩϵ��*/
    float wI_1;                     /*��һ�εĻ��ּ�Ȩϵ��*/
    float wD_1;                     /*��һ�ε�΢�ּ�Ȩϵ��*/ 
	float e;

	~PID_SingleNeuron(){};
    PID_SingleNeuron(float pr,float ir,float dr,float pw, float iw,float dw,float ko);
    float NeuronPid_control(float tar,float act);	
};

// ////λ��ʽPID
// class PID_position
// {
// private:
//     float kp;//����ϵ��
//     float ki;//����ϵ��
//     float kd;//΢��ϵ��
//     float target;//Ŀ��ֵ
//     float actual;//ʵ��ֵ
//     float e;//���
//     float e_pre;//��һ�����
//     float integral;//������
// public:
//     PID_position();
//     ~PID_position(){};
//     PID_position(float p,float i,float d);
//     float pid_control(float tar,float act);//ִ��PID����
//     void pid_show();//��ʾPID���������ڲ�����
// };

// ///����ʽPID
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