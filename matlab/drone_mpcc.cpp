/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 3){ 
      mexErrMsgTxt("This problem expects 3 right hand side argument(s) since you have defined 3 MexInput(s)");
    } 
 
    TIME autotime;
    double *mexinput0_temp = NULL; 
    if( !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || !(mxGetM(prhs[0])==1 && mxGetN(prhs[0])==1) ) { 
      mexErrMsgTxt("Input 0 must be a noncomplex scalar double.");
    } 
    mexinput0_temp = mxGetPr(prhs[0]); 
    double mexinput0 = *mexinput0_temp; 

    int mexinput1_count = 0;
    if (mxGetM(prhs[1]) == 1 && mxGetN(prhs[1]) >= 1) 
       mexinput1_count = mxGetN(prhs[1]);
    else if (mxGetM(prhs[1]) >= 1 && mxGetN(prhs[1]) == 1) 
       mexinput1_count = mxGetM(prhs[1]);
    else 
       mexErrMsgTxt("Input 1 must be a noncomplex double vector of dimension 1xY.");

    double *mexinput1_temp = NULL; 
    if( !mxIsDouble(prhs[1]) || mxIsComplex(prhs[1])) { 
      mexErrMsgTxt("Input 1 must be a noncomplex double vector of dimension 1xY.");
    } 
    mexinput1_temp = mxGetPr(prhs[1]); 
    DVector mexinput1(mexinput1_count);
    for( int i=0; i<mexinput1_count; ++i ){ 
        mexinput1(i) = mexinput1_temp[i];
    } 

    double *mexinput2_temp = NULL; 
    if( !mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]) ) { 
      mexErrMsgTxt("Input 2 must be a noncomplex double vector of dimension XxY.");
    } 
    mexinput2_temp = mxGetPr(prhs[2]); 
    DMatrix mexinput2(mxGetM(prhs[2]), mxGetN(prhs[2]));
    for( int i=0; i<mxGetN(prhs[2]); ++i ){ 
        for( int j=0; j<mxGetM(prhs[2]); ++j ){ 
           mexinput2(j,i) = mexinput2_temp[i*mxGetM(prhs[2]) + j];
        } 
    } 

    DifferentialState x;
    DifferentialState y;
    DifferentialState z;
    DifferentialState qw;
    DifferentialState qx;
    DifferentialState qy;
    DifferentialState qz;
    DifferentialState vx;
    DifferentialState vy;
    DifferentialState vz;
    DifferentialState T;
    DifferentialState th;
    Control vT;
    Control wx;
    Control wy;
    Control wz;
    Control vth;
    Function acadodata_f2;
    acadodata_f2 << ((-5.77350269189625842081e-01*th+x)*5.77350269189625842081e-01+(-5.77350269189625842081e-01*th+y)*5.77350269189625842081e-01+(-5.77350269189625842081e-01*th+z)*5.77350269189625842081e-01)*5.77350269189625842081e-01;
    acadodata_f2 << ((-5.77350269189625842081e-01*th+x)*5.77350269189625842081e-01+(-5.77350269189625842081e-01*th+y)*5.77350269189625842081e-01+(-5.77350269189625842081e-01*th+z)*5.77350269189625842081e-01)*5.77350269189625842081e-01;
    acadodata_f2 << ((-5.77350269189625842081e-01*th+x)*5.77350269189625842081e-01+(-5.77350269189625842081e-01*th+y)*5.77350269189625842081e-01+(-5.77350269189625842081e-01*th+z)*5.77350269189625842081e-01)*5.77350269189625842081e-01;
    acadodata_f2 << (-((-5.77350269189625842081e-01*th+x)*5.77350269189625842081e-01+(-5.77350269189625842081e-01*th+y)*5.77350269189625842081e-01+(-5.77350269189625842081e-01*th+z)*5.77350269189625842081e-01)*5.77350269189625842081e-01-5.77350269189625842081e-01*th+x);
    acadodata_f2 << (-((-5.77350269189625842081e-01*th+x)*5.77350269189625842081e-01+(-5.77350269189625842081e-01*th+y)*5.77350269189625842081e-01+(-5.77350269189625842081e-01*th+z)*5.77350269189625842081e-01)*5.77350269189625842081e-01-5.77350269189625842081e-01*th+y);
    acadodata_f2 << (-((-5.77350269189625842081e-01*th+x)*5.77350269189625842081e-01+(-5.77350269189625842081e-01*th+y)*5.77350269189625842081e-01+(-5.77350269189625842081e-01*th+z)*5.77350269189625842081e-01)*5.77350269189625842081e-01-5.77350269189625842081e-01*th+z);
    acadodata_f2 << wx;
    acadodata_f2 << wy;
    acadodata_f2 << wz;
    DVector acadodata_v1(9);
    acadodata_v1(0) = 0;
    acadodata_v1(1) = 0;
    acadodata_v1(2) = 0;
    acadodata_v1(3) = 0;
    acadodata_v1(4) = 0;
    acadodata_v1(5) = 0;
    acadodata_v1(6) = 0;
    acadodata_v1(7) = 0;
    acadodata_v1(8) = 0;
    DVector acadodata_v2(9);
    acadodata_v2(0) = 0;
    acadodata_v2(1) = 0;
    acadodata_v2(2) = 0;
    acadodata_v2(3) = 0;
    acadodata_v2(4) = 0;
    acadodata_v2(5) = 0;
    acadodata_v2(6) = 0;
    acadodata_v2(7) = 0;
    acadodata_v2(8) = 0;
    DVector acadodata_v3(12);
    acadodata_v3(0) = 0;
    acadodata_v3(1) = 0;
    acadodata_v3(2) = 0;
    acadodata_v3(3) = 0;
    acadodata_v3(4) = 0;
    acadodata_v3(5) = 0;
    acadodata_v3(6) = 0;
    acadodata_v3(7) = 0;
    acadodata_v3(8) = 0;
    acadodata_v3(9) = 0;
    acadodata_v3(10) = 0;
    acadodata_v3(11) = 0;
    DVector acadodata_v4(5);
    acadodata_v4(0) = 0;
    acadodata_v4(1) = 0;
    acadodata_v4(2) = 0;
    acadodata_v4(3) = 0;
    acadodata_v4(4) = -200;
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(x) == vx;
    acadodata_f1 << dot(y) == vy;
    acadodata_f1 << dot(z) == vz;
    acadodata_f1 << dot(qw) == (((-qx)*wx+(-qy)*wy+(-qz)*wz)/2.00000000000000000000e+00-(-1.00000000000000000000e+00+qw*qw+qx*qx+qy*qy+qz*qz)*qw);
    acadodata_f1 << dot(qx) == (((-qz)*wy+qw*wx+qy*wz)/2.00000000000000000000e+00-(-1.00000000000000000000e+00+qw*qw+qx*qx+qy*qy+qz*qz)*qx);
    acadodata_f1 << dot(qy) == (((-qx)*wz+qw*wy+qz*wx)/2.00000000000000000000e+00-(-1.00000000000000000000e+00+qw*qw+qx*qx+qy*qy+qz*qz)*qy);
    acadodata_f1 << dot(qz) == (((-qy)*wx+qw*wz+qx*wy)/2.00000000000000000000e+00-(-1.00000000000000000000e+00+qw*qw+qx*qx+qy*qy+qz*qz)*qz);
    acadodata_f1 << dot(vx) == ((-qw)*(-qy)+(-qx)*(-qz)+(-qx)*(-qz)+qw*qy)*T;
    acadodata_f1 << dot(vy) == ((-qx)*qw+(-qx)*qw+(-qy)*(-qz)+(-qy)*(-qz))*T;
    acadodata_f1 << dot(vz) == (((-(-qx))*(-qx)+(-qy)*qy+(-qz)*(-qz)+qw*qw)*T+(-9.81000000000000049738e+00));
    acadodata_f1 << dot(T) == vT;
    acadodata_f1 << dot(th) == vth;

    OCP ocp1(0, 0.3, 30);
    ocp1.minimizeLSQ(mexinput2, acadodata_f2, acadodata_v2);
    ocp1.minimizeLSQLinearTerms(acadodata_v3, acadodata_v4);
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo(0.00000000000000000000e+00 <= T <= 1.50000000000000000000e+01);
    ocp1.subjectTo((-5.00000000000000000000e+00) <= vT <= 5.00000000000000000000e+00);
    ocp1.subjectTo((-1.00000000000000000000e+01) <= wx <= 1.00000000000000000000e+01);
    ocp1.subjectTo((-1.00000000000000000000e+01) <= wy <= 1.00000000000000000000e+01);
    ocp1.subjectTo((-1.00000000000000000000e+01) <= wz <= 1.00000000000000000000e+01);
    ocp1.subjectTo(0.00000000000000000000e+00 <= vth <= 1.00000000000000000000e+00);


    RealTimeAlgorithm algo1(ocp1, 0.01);
    algo1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    algo1.set( MAX_NUM_ITERATIONS, 2 );
    algo1.set( PRINTLEVEL, 0 );
    algo1.set( PRINT_COPYRIGHT, 0 );

    Controller controller1( algo1 );
    controller1.init(mexinput0, mexinput1);
    controller1.step(mexinput0, mexinput1);

    const char* outputFieldNames[] = {"U", "P"}; 
    plhs[0] = mxCreateStructMatrix( 1,1,2,outputFieldNames ); 
    mxArray *OutU = NULL;
    double  *outU = NULL;
    OutU = mxCreateDoubleMatrix( 1,controller1.getNU(),mxREAL ); 
    outU = mxGetPr( OutU );
    DVector vec_outU; 
    controller1.getU(vec_outU); 
    for( int i=0; i<vec_outU.getDim(); ++i ){ 
        outU[i] = vec_outU(i); 
    } 

    mxArray *OutP = NULL;
    double  *outP = NULL;
    OutP = mxCreateDoubleMatrix( 1,controller1.getNP(),mxREAL ); 
    outP = mxGetPr( OutP );
    DVector vec_outP; 
    controller1.getP(vec_outP); 
    for( int i=0; i<vec_outP.getDim(); ++i ){ 
        outP[i] = vec_outP(i); 
    } 

    mxSetField( plhs[0],0,"U",OutU );
    mxSetField( plhs[0],0,"P",OutP );


    clearAllStaticCounters( ); 
 
} 

