rem code generator build script
@echo off
setlocal

path=%path%;C:\Windows\Microsoft.NET\Framework64\v4.0.30319

cd D:\ACADOtoolkit\ACADOtoolkit\build\examples
msbuild getting_started_simple_mpc.vcxproj /p:configuration=release
if %errorlevel% neq 0 goto end

cd D:\ACADOtoolkit\ACADOtoolkit\examples\getting_started\Release
simple_mpc
cd D:\ACADOtoolkit\ACADOtoolkit\examples\getting_started\Release\simple_mpc_export
copy /y acado_aux*.h D:\Projects\robomower\robomow_rc304\pathtracking\mpc_acado
copy /y acado_aux*.c D:\Projects\robomower\robomow_rc304\pathtracking\mpc_acado
copy /y acado_common.h D:\Projects\robomower\robomow_rc304\pathtracking\mpc_acado
copy /y acado_integrator.c D:\Projects\robomower\robomow_rc304\pathtracking\mpc_acado
copy /y acado_qp* D:\Projects\robomower\robomow_rc304\pathtracking\mpc_acado
copy /y acado_solver.c D:\Projects\robomower\robomow_rc304\pathtracking\mpc_acado
cd D:\Projects\robomower\robomow_rc304\pathtracking\mpc_acado
python setup.py build install --force
cd D:\Projects\robomower\robomow_rc304\pathtracking\


:end
endlocal
