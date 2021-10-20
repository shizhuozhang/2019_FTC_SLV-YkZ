#include "single_neuron_pid.h"

/*位置PID*/
// PID_position::PID_position():kp(0),ki(0),kd(0),target(0),actual(0),integral(0)
// {
//     e=target-actual;
//     e_pre=e;
// }
// PID_position::PID_position(float p,float i,float d):kp(p),ki(i),kd(d),target(0),actual(0),integral(0)
// {
//    e=target-actual;
//    e_pre=e;
// }
// float PID_position::pid_control(float tar,float act)
// {
//     float u;
//     target=tar;
//     actual=act;
//     e=target-actual;
//     integral+=e;
//     u=kp*e+ki*integral+kd*(e-e_pre);
//     e_pre=e;
//     return u;
// }
// void PID_position::pid_show()
// {
//     using std::cout;
//     using std::endl;
//     cout<<"The infomation of this position PID controller is as following:"<<endl;
//     cout<<"       Kp="<<kp<<endl;
//     cout<<"       Ki="<<ki<<endl;
//     cout<<"       Kd="<<kd<<endl;
//     cout<<" integral="<<integral<<endl;
//     cout<<"   target="<<target<<endl;
//     cout<<"   actual="<<actual<<endl;
//     cout<<"        e="<<e<<endl;
//     cout<<"    e_pre="<<e_pre<<endl;
// }
// 
// 
// /*增量PID*/
// PID_incremental::PID_incremental():kp(0),ki(0),kd(0),e_pre_1(0),e_pre_2(0),target(0),actual(0)
// {
//    A=kp+ki+kd;
//    B=-2*kd-kp;
//    C=kd;
//    e=target-actual;
// }
// PID_incremental::PID_incremental(float p,float i,float d):kp(p),ki(i),kd(d),e_pre_1(0),e_pre_2(0),target(0),actual(0)
// {
//    A=kp+ki+kd;
//    B=-2*kd-kp;
//    C=kd;
//    e=target-actual;
// }
// float PID_incremental::pid_control(float tar,float act)
// {
//    float u_increment;
//    target=tar;
//    actual=act;
//    e=target-actual;
//    u_increment=A*e+B*e_pre_1+C*e_pre_2;
//    e_pre_2=e_pre_1;
//    e_pre_1=e;
//    return u_increment;
// }
// 
// void PID_incremental::pid_show()
// {
//     using std::cout;
//     using std::endl;
//     cout<<"The infomation of this incremental PID controller is as following:"<<endl;
//     cout<<"     Kp="<<kp<<endl;
//     cout<<"     Ki="<<ki<<endl;
//     cout<<"     Kd="<<kd<<endl;
//     cout<<" target="<<target<<endl;
//     cout<<" actual="<<actual<<endl;
//     cout<<"      e="<<e<<endl;
//     cout<<"e_pre_1="<<e_pre_1<<endl;
//     cout<<"e_pre_2="<<e_pre_2<<endl;
// }
// 



PID_SingleNeuron::PID_SingleNeuron(float pr,float ir,float dr,float pw, float iw,float dw,float ko):Prate(pr),Irate(ir),
							Drate(dr),wP_1(pw),wI_1(iw),wD_1(dw),Kout(ko),e_pre_1(0),e_pre_2(0),Ref_r(0),Act_y(0),u_1(0)
{
   e=Ref_r-Act_y;	
}

float PID_SingleNeuron::NeuronPid_control(float refer,float act)
{
   float u_out;
   float sabs;
   float wP,wI,wD,wP_temp,wI_temp,wD_temp;
   float x1,x2,x3;

   Ref_r=refer;
   Act_y=act;
   e=Ref_r-Act_y;

   x1= e - e_pre_1;
   x2= e;
   x3= e - 2* e_pre_1 + e_pre_2;

   wP_temp= wP_1 + Prate * e * u_1 * x1;
   wI_temp= wI_1 + Irate * e * u_1 * x2;
   wD_temp= wD_1 + Drate * e * u_1 * x3;

   sabs=fabs(wP_temp)+fabs(wI_temp)+fabs(wD_temp);
   wP= wP_temp/sabs;
   wI= wI_temp/sabs;
   wD= wD_temp/sabs;

   u_out= u_1 + Kout*(wP*x1 + wI*x2 + wD*x3);

   e_pre_2 = e_pre_1;
   e_pre_1 = e;
   u_1     = u_out;  
   wP_1    = wP_temp;
   wI_1    = wI_temp;
   wD_1    = wD_temp;
   
   return u_out;	
}


/* 单神经元PID初始化操作,需在对vPID对象的值进行修改前完成                     */
/* NEURALPID vPID，单神经元PID对象变量，实现数据交换与保存                    */
/* float vMax,float vMin，过程变量的最大最小值（量程范围）                    */
// void NeuralPIDInitialization(NEURALPID *vPID,float vMax,float vMin)
// 
// {
//   vPID->setpoint=vMin;                  /*设定值*/
// 
//   vPID->kcoef=0.12; /*神经元输出比例*/
// 
//   vPID->kp=0.4;                         /*比例学习速度*/
//   vPID->ki=0.35;                        /*积分学习速度*/
//   vPID->kd=0.4;                         /*微分学习速度*/
// 
//   vPID->lasterror=0.0;                  /*前一拍偏差*/
//   vPID->preerror=0.0;                   /*前两拍偏差*/
//   vPID->result=vMin;                    /*PID控制器结果*/
//   vPID->output=0.0;                     /*输出值，百分比*/
// 
//   vPID->maximum=vMax;                   /*输出值上限*/
//   vPID->minimum=vMin;                   /*输出值下限*/  
//   vPID->deadband=(vMax-vMin)*0.0005;    /*死区*/
// 
//   vPID->wp=0.10; /*比例加权系数*/
//   vPID->wi=0.10; /*积分加权系数*/
//   vPID->wd=0.10; /*微分加权系数*/
// }
/*单神经元学习规则函数*/

// static void NeureLearningRules(NEURALPID *vPID,float zk,float uk,float *xi)
// 
// {
// 
//   vPID->wi=vPID->wi+vPID->ki*zk*uk*xi[0];
// 
//   vPID->wp=vPID->wp+vPID->kp*zk*uk*xi[1];
// 
//   vPID->wd=vPID->wd+vPID->kd*zk*uk*xi[2];
// 
// }

/* 神经网络参数自整定PID控制器，以增量型方式实现                              */
/* NEURALPID vPID，神经网络PID对象变量，实现数据交换与保存                    */
/* float pv，过程测量值，对象响应的测量数据，用于控制反馈                     */

// void NeuralPID(NEURALPID *vPID,float pv)
// {
// float x[3];
//   float w[3];
//   float sabs;
//   float error;
//   float result;
//   float deltaResult;
// 
//   error=vPID->setpoint-pv;
//   result=vPID->result;
// 
//   if(fabs(error)>vPID->deadband)
// 
//   {
//     x[0]=error;
//     x[1]=error-vPID->lasterror;
//     x[2]=error-vPID->lasterror*2+vPID->preerror;
//   
//     sabs=fabs(vPID->wi)+fabs(vPID->wp)+fabs(vPID->wd);
// 
//     w[0]=vPID->wi/sabs;
// 
//     w[1]=vPID->wp/sabs;
// 
//     w[2]=vPID->wd/sabs;
//    
//     deltaResult=(w[0]*x[0]+w[1]*x[1]+w[2]*x[2])*vPID->kcoef;
// 
//     }
// 
//   else
//   {
//     deltaResult=0;
//   }
// 
//  
//   result=result+deltaResult;
// 
//   if(result>vPID->maximum)
//   {
//     result=vPID->maximum;
//   }
// 
//   if(result<vPID->minimum)
//   {
//     result=vPID->minimum;
//   }
// 
//   vPID->result=result;
//   vPID->output=(vPID->result-vPID->minimum)*100/(vPID->maximum-vPID->minimum);
// 
//   //单神经元学习
//   NeureLearningRules(vPID,error,result,x);
// 
//   vPID->preerror=vPID->lasterror;
//   vPID->lasterror=error;
// 
// }