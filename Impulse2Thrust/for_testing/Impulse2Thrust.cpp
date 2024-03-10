//
// MATLAB Compiler: 7.0.1 (R2019a)
// Date: Sun Nov  6 17:36:46 2022
// Arguments:
// "-B""macro_default""-W""cpplib:Impulse2Thrust,all""-T""link:lib""-d""D:\ÏîÄ¿0
// 901\»¯ÍÆ-¿¼ÂÇÖÊÁ¿dll\Impulse2Thrust\for_testing""-v""D:\ÏîÄ¿0901\»¯ÍÆ-¿¼ÂÇÖÊÁ
// ¿dll\Impulse2Thrust.m"
//

#define EXPORTING_Impulse2Thrust 1
#include "Impulse2Thrust.h"

static HMCRINSTANCE _mcr_inst = NULL;

#if defined( _MSC_VER) || defined(__LCC__) || defined(__MINGW64__)
#ifdef __LCC__
#undef EXTERN_C
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#define NOMINMAX
#include <windows.h>
#undef interface

static char path_to_dll[_MAX_PATH];

BOOL WINAPI DllMain(HINSTANCE hInstance, DWORD dwReason, void *pv)
{
    if (dwReason == DLL_PROCESS_ATTACH)
    {
        if (GetModuleFileName(hInstance, path_to_dll, _MAX_PATH) == 0)
            return FALSE;
    }
    else if (dwReason == DLL_PROCESS_DETACH)
    {
    }
    return TRUE;
}
#endif
#ifdef __cplusplus
extern "C" {
#endif

static int mclDefaultPrintHandler(const char *s)
{
    return mclWrite(1 /* stdout */, s, sizeof(char)*strlen(s));
}

#ifdef __cplusplus
} /* End extern "C" block */
#endif

#ifdef __cplusplus
extern "C" {
#endif

static int mclDefaultErrorHandler(const char *s)
{
    int written = 0;
    size_t len = 0;
    len = strlen(s);
    written = mclWrite(2 /* stderr */, s, sizeof(char)*len);
    if (len > 0 && s[ len-1 ] != '\n')
        written += mclWrite(2 /* stderr */, "\n", sizeof(char));
    return written;
}

#ifdef __cplusplus
} /* End extern "C" block */
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_Impulse2Thrust_C_API
#define LIB_Impulse2Thrust_C_API /* No special import/export declaration */
#endif

LIB_Impulse2Thrust_C_API 
bool MW_CALL_CONV Impulse2ThrustInitializeWithHandlers(
    mclOutputHandlerFcn error_handler,
    mclOutputHandlerFcn print_handler)
{
    int bResult = 0;
    if (_mcr_inst != NULL)
        return true;
    if (!mclmcrInitialize())
        return false;
    if (!GetModuleFileName(GetModuleHandle("Impulse2Thrust"), path_to_dll, _MAX_PATH))
        return false;
    {
        mclCtfStream ctfStream = 
            mclGetEmbeddedCtfStream(path_to_dll);
        if (ctfStream) {
            bResult = mclInitializeComponentInstanceEmbedded(&_mcr_inst,
                                                             error_handler, 
                                                             print_handler,
                                                             ctfStream);
            mclDestroyStream(ctfStream);
        } else {
            bResult = 0;
        }
    }  
    if (!bResult)
    return false;
    return true;
}

LIB_Impulse2Thrust_C_API 
bool MW_CALL_CONV Impulse2ThrustInitialize(void)
{
    return Impulse2ThrustInitializeWithHandlers(mclDefaultErrorHandler, 
                                              mclDefaultPrintHandler);
}

LIB_Impulse2Thrust_C_API 
void MW_CALL_CONV Impulse2ThrustTerminate(void)
{
    if (_mcr_inst != NULL)
        mclTerminateInstance(&_mcr_inst);
}

LIB_Impulse2Thrust_C_API 
void MW_CALL_CONV Impulse2ThrustPrintStackTrace(void) 
{
    char** stackTrace;
    int stackDepth = mclGetStackTrace(&stackTrace);
    int i;
    for(i=0; i<stackDepth; i++)
    {
        mclWrite(2 /* stderr */, stackTrace[i], sizeof(char)*strlen(stackTrace[i]));
        mclWrite(2 /* stderr */, "\n", sizeof(char)*strlen("\n"));
    }
    mclFreeStackTrace(&stackTrace, stackDepth);
}


LIB_Impulse2Thrust_C_API 
bool MW_CALL_CONV mlxImpulse2Thrust(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[])
{
    return mclFeval(_mcr_inst, "Impulse2Thrust", nlhs, plhs, nrhs, prhs);
}

LIB_Impulse2Thrust_CPP_API 
void MW_CALL_CONV Impulse2Thrust(int nargout, mwArray& Thrust_t, mwArray& Thrust_angle, 
                                 mwArray& Mass, const mwArray& rv_c, const mwArray& p, 
                                 const mwArray& Thrust_F, const mwArray& mass, const 
                                 mwArray& Isp)
{
    mclcppMlfFeval(_mcr_inst, "Impulse2Thrust", nargout, 3, 5, &Thrust_t, &Thrust_angle, &Mass, &rv_c, &p, &Thrust_F, &mass, &Isp);
}

