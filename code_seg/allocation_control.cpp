void CRockSim:: Allocation_Control(void)
{
	double  Pxj1,Pxj2,Pzt1,Pzt2,Pzt3,Pzt4;
	double	Pxj,Pzt;
	double	Xz;
	double  A_zt_x,A_xj_x,A_zt_yz,A_xj_yz;
	double pitchDtfz, yawDtfz, rollDtfz, pitchDtfx, yawDtfx, rollDtfx;
	
	pitchDtfz = m_dtf.P.pitchDtfz;
    yawDtfz = m_dtf.P.yawDtfz;
	rollDtfz = m_dtf.P.rollDtfz;
	
	pitchDtfx = m_dtf.P.pitchDtfx;
	yawDtfx = m_dtf.P.yawDtfx;
	rollDtfx = m_dtf.P.rollDtfx;

	//芯级推力、助推级推力
	m_nonlinmodel->Insert1(&m_pr.P.T);	
	Pxj=m_nonlinmodel->pUqd.Px.Pxj;
	Pzt=m_nonlinmodel->pUqd.Px.Pzt;
	
	//质心
	m_MJ->Insert1(&m_pr.P.T);	
	Xz=m_MJ->pUqd.Px.xcg;
	
	Pxj1=TLXS[0];
	Pxj2=TLXS[1];
	Pzt1=TLXS[2];
	Pzt2=TLXS[3];
	Pzt3=TLXS[4];
	Pzt4=TLXS[5];

	A_zt_x=Pzt*MS::Rzt;//先算一下基础力矩
	A_xj_x=Pxj*MS::Rxj;	
	A_zt_yz=(Pzt*(MS::Xrzt-Xz));
	A_xj_yz=(Pxj*(MS::Xrxj-Xz));
	
	double	Vb[]={-A_zt_x*Pzt1,-A_zt_x*Pzt2,-A_zt_x*Pzt3,-A_zt_x*Pzt4,
				  -A_xj_x*Pxj1,-A_xj_x*Pxj2,-A_xj_x*Pxj2,-A_xj_x*Pxj1,  
			   	  Pzt1*A_zt_yz,	0,			-Pzt3*A_zt_yz,0,
		          Pxj1*A_xj_yz,	0,			-Pxj2*A_xj_yz,0,  
		          0,			Pzt2*A_zt_yz,0,				-Pzt4*A_zt_yz,
		          0,			Pxj2*A_xj_yz,0,				-Pxj1*A_xj_yz	};
	CMatrix	testb=	CMatrix(Vb,3,8);
	
	double Mxc,Myc,Mzc;//三个通道控制力矩

	Mxc=-4*A_zt_x*rollDtfz-4*A_xj_x*rollDtfx;
	Myc=-2*A_zt_yz*yawDtfz-2*A_xj_yz*yawDtfx;
	Mzc=-2*A_zt_yz*pitchDtfz-2*A_xj_yz*pitchDtfx;

	Coffset.F.expMx=Mxc;
	Coffset.F.expMy=Myc;
	Coffset.F.expMz=Mzc;

    double	Vv[]={Mxc,Myc,Mzc};
	CMatrix testv=	CMatrix(Vv,3,1);
	Allocation Rudder(testv,testb);
	//	CMatrix Pseudo=	Rudder.pinv();
	CMatrix Pseudo=	Rudder.control_allocation();

	//摆角赋值
	m_dtf.P.dtfz[0] = Pseudo(0,0);
	m_dtf.P.dtfz[1] = Pseudo(1,0);
	m_dtf.P.dtfz[2] = Pseudo(2,0);
	m_dtf.P.dtfz[3] = Pseudo(3,0);
	
	m_dtf.P.dtfx[0] = Pseudo(4,0);
	m_dtf.P.dtfx[1] = Pseudo(5,0);
	m_dtf.P.dtfx[2] = Pseudo(6,0);
	m_dtf.P.dtfx[3] = Pseudo(7,0);

	bool ztsaturflag[4];
	bool xjsaturflag[4];

	int sanum=0;//饱和发动机数量
	//第二次伪逆
	//查找摆角饱和发动机
	for(int i=0;i<8;i++){
		if(i<4){
			if (fabs(m_dtf.P.dtfz[i]) > (8/MS::DEG)){
				m_dtf.P.dtfz[i] = sign(m_dtf.P.dtfz[i]) * (8/MS::DEG);
				ztsaturflag[i]=false;
				sanum++;
			}
			else
			ztsaturflag[i]=true;
		}
		else
		{
			if (fabs(m_dtf.P.dtfx[i-4]) > (6/MS::DEG)){
				m_dtf.P.dtfx[i-4] = sign(m_dtf.P.dtfx[i-4]) * (6/MS::DEG);
				xjsaturflag[i-4]=false;
				sanum++;
			}
			else
			xjsaturflag[i-4]=true;
		}
		
	}
	//效率矩阵和控制力矩重新赋值

	CMatrix	temco(3,8-sanum);
	int j=0;
	for(int k=0;k<4;k++){
		if(ztsaturflag[k]){
			temco(0,j)=testb(0,k);
			temco(1,j)=testb(1,k);
			temco(2,j)=testb(2,k);
			j++;
		}
		else{
			Mxc=Mxc-m_dtf.P.dtfz[k]*testb(0,k);
			Myc=Myc-m_dtf.P.dtfz[k]*testb(1,k);
			Mzc=Mzc-m_dtf.P.dtfz[k]*testb(2,k);
		}
	}
	for(int n=0;n<4;n++){
		if(xjsaturflag[n]){
			temco(0,j)=testb(0,n+4);
			temco(1,j)=testb(1,n+4);
			temco(2,j)=testb(2,n+4);
			j++;
		}
		else{
			Mxc=Mxc-m_dtf.P.dtfx[n]*testb(0,n+4);
			Myc=Myc-m_dtf.P.dtfx[n]*testb(1,n+4);
			Mzc=Mzc-m_dtf.P.dtfx[n]*testb(2,n+4);
		}
	}
	//第二次伪逆
    double	Vtwo[]={Mxc,Myc,Mzc};
	CMatrix testtwo=	CMatrix(Vtwo,3,1);
	Allocation Rudder_two(testtwo,temco);
	
	CMatrix Pseudo_two=	Rudder_two.control_allocation();
	//发动机摆角重新赋值
	int m=0;
	for(int zn=0;zn<4;zn++){
		if(ztsaturflag[zn]){
			m_dtf.P.dtfz[zn] = Pseudo_two(m,0);
			m++;
		}
	}
	for(int xn=0;xn<4;xn++){
		if(xjsaturflag[xn]){
			m_dtf.P.dtfx[xn] = Pseudo_two(m,0);
			m++;
		}
	}	
}
