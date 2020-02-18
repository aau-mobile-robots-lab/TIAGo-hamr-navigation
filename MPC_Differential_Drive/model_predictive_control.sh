# code generator build script

set -e

cd ~/ACADOtoolkit/build
make
cd ~/ACADOtoolkit/examples/getting_started/
./simple_mpc
cd ~/ACADOtoolkit/examples/getting_started/simple_mpc_export
cp acado_aux*.h ~/robomower/robomow_rc304/pathtracking/mpc_acado
cp acado_aux*.c ~/robomower/robomow_rc304/pathtracking/mpc_acado
cp acado_common.h ~/robomower/robomow_rc304/pathtracking/mpc_acado
cp acado_integrator.c ~/robomower/robomow_rc304/pathtracking/mpc_acado
cp acado_qp* ~/robomower/robomow_rc304/pathtracking/mpc_acado
cp acado_solver.c ~/robomower/robomow_rc304/pathtracking/mpc_acado
cd ~/robomower/robomow_rc304/pathtracking/mpc_acado
python3 setup.py build install --force
cd ~/robomower/robomow_rc304/pathtracking


