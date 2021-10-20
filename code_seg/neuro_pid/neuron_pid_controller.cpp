#include "single_neuron_pid.h"
#include <fstream.h>
#include <iomanip.h>
#include <string.h>

using namespace std;
#define ts 0.001
float yout,y_1,y_2,uout;
float vara;
float rin;
float run_time;
void model(int mod, int steps)
{	
	if(mod==1)
	{
		rin=1.0f;
	}	
	else
	{
		rin= sin(0.5 * 3.14 * ts *steps);	
	}
	vara= 1.2*(1-0.8*pow(3,(-0.1*steps)));
	yout=vara*y_1/(1+pow(y_1,2))+uout;
}

int main()
{	
	::ofstream piddata;
	piddata.open(".\\piddata.dat", ios::out|ios::trunc);
	std::string str1 ="ts(s)	uout	yout	wP	wI	wD	e";
	str1 +="\n";
	piddata<<str1.c_str();
	
	PID_SingleNeuron testpid(0.3,0.2,0.4,0.1,0.1,0.1,0.3);

	for(int N=1;N<5000;N++)
    {
        run_time= N*ts;
		model(2, N);
		uout=testpid.NeuronPid_control(rin,yout);
		y_2= y_1;
		y_1= yout;

		int width = 15;
		piddata.precision(5);
		piddata.flags(ios::right || ios::fixed);

		piddata << setw(width) << run_time;
		piddata << setw(width) <<uout;
		piddata << setw(width) <<yout;
		piddata << setw(width) <<testpid.wP_1;
		piddata << setw(width) <<testpid.wI_1;
		piddata << setw(width) <<testpid.wD_1;
		piddata << setw(width) <<testpid.e;
		piddata << endl;
    }

    return 0;
}

	//²âÊÔÔöÁ¿PID
//     PID_incremental pid1(0.35,0.65,0.005);
//     float pid_increment=0.0;
//     pid1.pid_show();
//     cout<<"target="<<target<<endl;
//     for(;N>0;N--)
//     {
//         pid_increment=pid1.pid_control(target,actual);
//         actual+=pid_increment;
//         cout<<"N="<<50-N<<"   actual="<<actual<<endl;
//     }
//     pid1.pid_show();

     //²âÊÔÎ»ÖÃPID
//     PID_position pid2(0.59,0.35,0.002);
//     pid2.pid_show();
//     cout<<"target="<<target<<endl;
//     N=100;
//     for(;N>0;N--)
//     {
//         actual=pid2.pid_control(target,actual);
//         cout<<"N="<<100-N<<"   actual="<<actual<<endl;
//     }
//     pid2.pid_show();