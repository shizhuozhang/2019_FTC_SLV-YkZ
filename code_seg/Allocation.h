#include "CMatrix.h"


class Allocation
{
private:
	CMatrix vd;	            //�����������
	CMatrix B;				//����Ч�ʾ���
	CMatrix U;				//���ػ���˸���ڽ�		
	
public:
	

	//constructor
	Allocation	(const CMatrix& mat1,const CMatrix& mat2);
	
	Allocation	(void);												//default constructor
	~Allocation();		
	
	/***********��������ز���-Overloaded Part*****************/

	void	getmatrix(const CMatrix& mat1,const CMatrix& mat2);			//����ֵ

	CMatrix Allocation::pinv();
	CMatrix Allocation::control_allocation();
	


};