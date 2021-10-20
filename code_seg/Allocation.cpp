#include "stdafx.h"
#include <stdlib.h>
#include <Math.h>
#include <iostream.h>
#include <assert.h>
#include "CMatrix.h"
#include "Allocation.h"

//=============================================================================
//���캯��
//=============================================================================
//-----------------constructor1------------------------
Allocation::Allocation(const CMatrix& mat1,const CMatrix& mat2)
{
	vd=mat1;
	B=mat2;
}
//----------------constructor2---------------------------
Allocation::Allocation(void)
{
}
//----------------constructor3----------------------------


//============================================================================
//��������
//============================================================================
Allocation::~Allocation()
{
}

//============================================================================
//�������ֵ
//============================================================================
void Allocation::getmatrix(const CMatrix& mat1,const CMatrix& mat2)
{	
	vd=mat1;
	B=mat2;
	return ;
}


//============================================================================
//��ȡα�����      
//============================================================================
CMatrix Allocation::pinv()
{
	
	CMatrix AT;	                //ת��Ч�ʾ���
	CMatrix AAT;	            //Ч�ʾ�����ת��Ч�ʾ���˻�����
	CMatrix Res;	            //Ч�ʾ����α�����
	AT=B.transpose();
	AAT=B*AT;
	AAT.Inv();
	Res=AT*AAT;
	return Res;
}

CMatrix Allocation::control_allocation()
{

	CMatrix res;	                //���Ʒ����ÿ��ִ����������
	res=pinv()*vd;
	U=res;
	return res;
}


