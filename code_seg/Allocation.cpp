#include "stdafx.h"
#include <stdlib.h>
#include <Math.h>
#include <iostream.h>
#include <assert.h>
#include "CMatrix.h"
#include "Allocation.h"

//=============================================================================
//构造函数
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
//析构函数
//============================================================================
Allocation::~Allocation()
{
}

//============================================================================
//分配矩阵赋值
//============================================================================
void Allocation::getmatrix(const CMatrix& mat1,const CMatrix& mat2)
{	
	vd=mat1;
	B=mat2;
	return ;
}


//============================================================================
//求取伪逆矩阵      
//============================================================================
CMatrix Allocation::pinv()
{
	
	CMatrix AT;	                //转置效率矩阵
	CMatrix AAT;	            //效率矩阵与转置效率矩阵乘积的逆
	CMatrix Res;	            //效率矩阵的伪逆矩阵
	AT=B.transpose();
	AAT=B*AT;
	AAT.Inv();
	Res=AT*AAT;
	return Res;
}

CMatrix Allocation::control_allocation()
{

	CMatrix res;	                //控制分配后每个执行器控制量
	res=pinv()*vd;
	U=res;
	return res;
}


