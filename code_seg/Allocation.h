#include "CMatrix.h"


class Allocation
{
private:
	CMatrix vd;	            //虚拟控制力矩
	CMatrix B;				//控制效率矩阵
	CMatrix U;				//运载火箭八个舵摆角		
	
public:
	

	//constructor
	Allocation	(const CMatrix& mat1,const CMatrix& mat2);
	
	Allocation	(void);												//default constructor
	~Allocation();		
	
	/***********运算符重载部分-Overloaded Part*****************/

	void	getmatrix(const CMatrix& mat1,const CMatrix& mat2);			//矩阵赋值

	CMatrix Allocation::pinv();
	CMatrix Allocation::control_allocation();
	


};