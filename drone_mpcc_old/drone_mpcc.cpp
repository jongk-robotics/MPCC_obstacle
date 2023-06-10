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

    if (nrhs != 11){ 
      mexErrMsgTxt("This problem expects 11 right hand side argument(s) since you have defined 11 MexInput(s)");
    } 
 
    TIME autotime;
    double *mexinput0_temp = NULL; 
    if( !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || !(mxGetM(prhs[0])==1 && mxGetN(prhs[0])==1) ) { 
      mexErrMsgTxt("Input 0 must be a noncomplex scalar double.");
    } 
    mexinput0_temp = mxGetPr(prhs[0]); 
    double mexinput0 = *mexinput0_temp; 

    double *mexinput1_temp = NULL; 
    if( !mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) || !(mxGetM(prhs[1])==1 && mxGetN(prhs[1])==1) ) { 
      mexErrMsgTxt("Input 1 must be a noncomplex scalar double.");
    } 
    mexinput1_temp = mxGetPr(prhs[1]); 
    double mexinput1 = *mexinput1_temp; 

    double *mexinput2_temp = NULL; 
    if( !mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]) || !(mxGetM(prhs[2])==1 && mxGetN(prhs[2])==1) ) { 
      mexErrMsgTxt("Input 2 must be a noncomplex scalar double.");
    } 
    mexinput2_temp = mxGetPr(prhs[2]); 
    double mexinput2 = *mexinput2_temp; 

    double *mexinput3_temp = NULL; 
    if( !mxIsDouble(prhs[3]) || mxIsComplex(prhs[3]) || !(mxGetM(prhs[3])==1 && mxGetN(prhs[3])==1) ) { 
      mexErrMsgTxt("Input 3 must be a noncomplex scalar double.");
    } 
    mexinput3_temp = mxGetPr(prhs[3]); 
    double mexinput3 = *mexinput3_temp; 

    double *mexinput4_temp = NULL; 
    if( !mxIsDouble(prhs[4]) || mxIsComplex(prhs[4]) || !(mxGetM(prhs[4])==1 && mxGetN(prhs[4])==1) ) { 
      mexErrMsgTxt("Input 4 must be a noncomplex scalar double.");
    } 
    mexinput4_temp = mxGetPr(prhs[4]); 
    double mexinput4 = *mexinput4_temp; 

    double *mexinput5_temp = NULL; 
    if( !mxIsDouble(prhs[5]) || mxIsComplex(prhs[5]) || !(mxGetM(prhs[5])==1 && mxGetN(prhs[5])==1) ) { 
      mexErrMsgTxt("Input 5 must be a noncomplex scalar double.");
    } 
    mexinput5_temp = mxGetPr(prhs[5]); 
    double mexinput5 = *mexinput5_temp; 

    double *mexinput6_temp = NULL; 
    if( !mxIsDouble(prhs[6]) || mxIsComplex(prhs[6]) || !(mxGetM(prhs[6])==1 && mxGetN(prhs[6])==1) ) { 
      mexErrMsgTxt("Input 6 must be a noncomplex scalar double.");
    } 
    mexinput6_temp = mxGetPr(prhs[6]); 
    double mexinput6 = *mexinput6_temp; 

    double *mexinput7_temp = NULL; 
    if( !mxIsDouble(prhs[7]) || mxIsComplex(prhs[7]) || !(mxGetM(prhs[7])==1 && mxGetN(prhs[7])==1) ) { 
      mexErrMsgTxt("Input 7 must be a noncomplex scalar double.");
    } 
    mexinput7_temp = mxGetPr(prhs[7]); 
    double mexinput7 = *mexinput7_temp; 

    double *mexinput8_temp = NULL; 
    if( !mxIsDouble(prhs[8]) || mxIsComplex(prhs[8]) || !(mxGetM(prhs[8])==1 && mxGetN(prhs[8])==1) ) { 
      mexErrMsgTxt("Input 8 must be a noncomplex scalar double.");
    } 
    mexinput8_temp = mxGetPr(prhs[8]); 
    double mexinput8 = *mexinput8_temp; 

    double *mexinput9_temp = NULL; 
    if( !mxIsDouble(prhs[9]) || mxIsComplex(prhs[9]) || !(mxGetM(prhs[9])==1 && mxGetN(prhs[9])==1) ) { 
      mexErrMsgTxt("Input 9 must be a noncomplex scalar double.");
    } 
    mexinput9_temp = mxGetPr(prhs[9]); 
    double mexinput9 = *mexinput9_temp; 

    double *mexinput10_temp = NULL; 
    if( !mxIsDouble(prhs[10]) || mxIsComplex(prhs[10]) || !(mxGetM(prhs[10])==1 && mxGetN(prhs[10])==1) ) { 
      mexErrMsgTxt("Input 10 must be a noncomplex scalar double.");
    } 
    mexinput10_temp = mxGetPr(prhs[10]); 
    double mexinput10 = *mexinput10_temp; 

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
    DifferentialState t;
    Control T;
    Control wx;
    Control wy;
    Control wz;
    Control vt;
    DiscretizedDifferentialEquation acadodata_f1(0.01);
    acadodata_f1 << next(x) == (1.00000000000000002082e-02*vx+x);
    acadodata_f1 << next(y) == (1.00000000000000002082e-02*vy+y);
    acadodata_f1 << next(z) == (1.00000000000000002082e-02*vz+z);
    acadodata_f1 << next(qw) == ((((-qx)*wx+(-qy)*wy+(-qz)*wz)/2.00000000000000000000e+00-(-1.00000000000000000000e+00+qw*qw+qx*qx+qy*qy+qz*qz)*qw)*1.00000000000000002082e-02+qw);
    acadodata_f1 << next(qx) == ((((-qz)*wy+qw*wx+qy*wz)/2.00000000000000000000e+00-(-1.00000000000000000000e+00+qw*qw+qx*qx+qy*qy+qz*qz)*qx)*1.00000000000000002082e-02+qx);
    acadodata_f1 << next(qy) == ((((-qx)*wz+qw*wy+qz*wx)/2.00000000000000000000e+00-(-1.00000000000000000000e+00+qw*qw+qx*qx+qy*qy+qz*qz)*qy)*1.00000000000000002082e-02+qy);
    acadodata_f1 << next(qz) == ((((-qy)*wx+qw*wz+qx*wy)/2.00000000000000000000e+00-(-1.00000000000000000000e+00+qw*qw+qx*qx+qy*qy+qz*qz)*qz)*1.00000000000000002082e-02+qz);
    acadodata_f1 << next(vx) == (((-qw)*(-qy)+(-qx)*(-qz)+(-qx)*(-qz)+qw*qy)*T*1.00000000000000002082e-02+vx);
    acadodata_f1 << next(vy) == (((-qx)*qw+(-qx)*qw+(-qy)*(-qz)+(-qy)*(-qz))*T*1.00000000000000002082e-02+vy);
    acadodata_f1 << next(vz) == ((((-(-qx))*(-qx)+(-qy)*qy+(-qz)*(-qz)+qw*qw)*T+(-9.81000000000000049738e+00))*1.00000000000000002082e-02+vz);
    acadodata_f1 << next(t) == (1.00000000000000002082e-02*vt+t);

    OCP ocp1(0, 0.3, 30);
    ocp1.minimizeLagrangeTerm(((-t+z)*(-t+z)*1.00000000000000000000e+01+1.00000000000000000000e+01*x*x+1.00000000000000000000e+01*y*y-vt));
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo(AT_START, t == mexinput10);
    ocp1.subjectTo(AT_START, x == mexinput0);
    ocp1.subjectTo(AT_START, y == mexinput1);
    ocp1.subjectTo(AT_START, z == mexinput2);
    ocp1.subjectTo(AT_START, qw == mexinput3);
    ocp1.subjectTo(AT_START, qx == mexinput4);
    ocp1.subjectTo(AT_START, qy == mexinput5);
    ocp1.subjectTo(AT_START, qz == mexinput6);
    ocp1.subjectTo(AT_START, vx == mexinput7);
    ocp1.subjectTo(AT_START, vy == mexinput8);
    ocp1.subjectTo(AT_START, vz == mexinput9);
    ocp1.subjectTo(0.00000000000000000000e+00 <= T <= 1.50000000000000000000e+01);
    ocp1.subjectTo(0.00000000000000000000e+00 <= vt <= 1.00000000000000000000e+00);


    OptimizationAlgorithm algo1(ocp1);
    returnValue returnvalue = algo1.solve();

    VariablesGrid out_states; 
    VariablesGrid out_parameters; 
    VariablesGrid out_controls; 
    VariablesGrid out_disturbances; 
    VariablesGrid out_algstates; 
    algo1.getDifferentialStates(out_states);
    algo1.getControls(out_controls);
    const char* outputFieldNames[] = {"STATES", "CONTROLS", "PARAMETERS", "DISTURBANCES", "ALGEBRAICSTATES", "CONVERGENCE_ACHIEVED"}; 
    plhs[0] = mxCreateStructMatrix( 1,1,6,outputFieldNames ); 
    mxArray *OutS = NULL;
    double  *outS = NULL;
    OutS = mxCreateDoubleMatrix( out_states.getNumPoints(),1+out_states.getNumValues(),mxREAL ); 
    outS = mxGetPr( OutS );
    for( int i=0; i<out_states.getNumPoints(); ++i ){ 
      outS[0*out_states.getNumPoints() + i] = out_states.getTime(i); 
      for( int j=0; j<out_states.getNumValues(); ++j ){ 
        outS[(1+j)*out_states.getNumPoints() + i] = out_states(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"STATES",OutS );
    mxArray *OutC = NULL;
    double  *outC = NULL;
    OutC = mxCreateDoubleMatrix( out_controls.getNumPoints(),1+out_controls.getNumValues(),mxREAL ); 
    outC = mxGetPr( OutC );
    for( int i=0; i<out_controls.getNumPoints(); ++i ){ 
      outC[0*out_controls.getNumPoints() + i] = out_controls.getTime(i); 
      for( int j=0; j<out_controls.getNumValues(); ++j ){ 
        outC[(1+j)*out_controls.getNumPoints() + i] = out_controls(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"CONTROLS",OutC );
    mxArray *OutP = NULL;
    double  *outP = NULL;
    OutP = mxCreateDoubleMatrix( out_parameters.getNumPoints(),1+out_parameters.getNumValues(),mxREAL ); 
    outP = mxGetPr( OutP );
    for( int i=0; i<out_parameters.getNumPoints(); ++i ){ 
      outP[0*out_parameters.getNumPoints() + i] = out_parameters.getTime(i); 
      for( int j=0; j<out_parameters.getNumValues(); ++j ){ 
        outP[(1+j)*out_parameters.getNumPoints() + i] = out_parameters(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"PARAMETERS",OutP );
    mxArray *OutW = NULL;
    double  *outW = NULL;
    OutW = mxCreateDoubleMatrix( out_disturbances.getNumPoints(),1+out_disturbances.getNumValues(),mxREAL ); 
    outW = mxGetPr( OutW );
    for( int i=0; i<out_disturbances.getNumPoints(); ++i ){ 
      outW[0*out_disturbances.getNumPoints() + i] = out_disturbances.getTime(i); 
      for( int j=0; j<out_disturbances.getNumValues(); ++j ){ 
        outW[(1+j)*out_disturbances.getNumPoints() + i] = out_disturbances(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"DISTURBANCES",OutW );
    mxArray *OutZ = NULL;
    double  *outZ = NULL;
    OutZ = mxCreateDoubleMatrix( out_algstates.getNumPoints(),1+out_algstates.getNumValues(),mxREAL ); 
    outZ = mxGetPr( OutZ );
    for( int i=0; i<out_algstates.getNumPoints(); ++i ){ 
      outZ[0*out_algstates.getNumPoints() + i] = out_algstates.getTime(i); 
      for( int j=0; j<out_algstates.getNumValues(); ++j ){ 
        outZ[(1+j)*out_algstates.getNumPoints() + i] = out_algstates(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"ALGEBRAICSTATES",OutZ );
    mxArray *OutConv = NULL;
    if ( returnvalue == SUCCESSFUL_RETURN ) { OutConv = mxCreateDoubleScalar( 1 ); }else{ OutConv = mxCreateDoubleScalar( 0 ); } 
    mxSetField( plhs[0],0,"CONVERGENCE_ACHIEVED",OutConv );


    clearAllStaticCounters( ); 
 
} 

