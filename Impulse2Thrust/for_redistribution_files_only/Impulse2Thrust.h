//
// MATLAB Compiler: 7.0.1 (R2019a)
// Date: Sun Nov  6 17:36:46 2022
// Arguments:
// "-B""macro_default""-W""cpplib:Impulse2Thrust,all""-T""link:lib""-d""D:\ÏîÄ¿0
// 901\»¯ÍÆ-¿¼ÂÇÖÊÁ¿dll\Impulse2Thrust\for_testing""-v""D:\ÏîÄ¿0901\»¯ÍÆ-¿¼ÂÇÖÊÁ
// ¿dll\Impulse2Thrust.m"
//

#ifndef __Impulse2Thrust_h
#define __Impulse2Thrust_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_Impulse2Thrust_C_API 
#define LIB_Impulse2Thrust_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_Impulse2Thrust_C_API 
bool MW_CALL_CONV Impulse2ThrustInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_Impulse2Thrust_C_API 
bool MW_CALL_CONV Impulse2ThrustInitialize(void);

extern LIB_Impulse2Thrust_C_API 
void MW_CALL_CONV Impulse2ThrustTerminate(void);

extern LIB_Impulse2Thrust_C_API 
void MW_CALL_CONV Impulse2ThrustPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_Impulse2Thrust_C_API 
bool MW_CALL_CONV mlxImpulse2Thrust(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_Impulse2Thrust
#define PUBLIC_Impulse2Thrust_CPP_API __declspec(dllexport)
#else
#define PUBLIC_Impulse2Thrust_CPP_API __declspec(dllimport)
#endif

#define LIB_Impulse2Thrust_CPP_API PUBLIC_Impulse2Thrust_CPP_API

#else

#if !defined(LIB_Impulse2Thrust_CPP_API)
#if defined(LIB_Impulse2Thrust_C_API)
#define LIB_Impulse2Thrust_CPP_API LIB_Impulse2Thrust_C_API
#else
#define LIB_Impulse2Thrust_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_Impulse2Thrust_CPP_API void MW_CALL_CONV Impulse2Thrust(int nargout, mwArray& Thrust_t, mwArray& Thrust_angle, mwArray& Mass, const mwArray& rv_c, const mwArray& p, const mwArray& Thrust_F, const mwArray& mass, const mwArray& Isp);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
