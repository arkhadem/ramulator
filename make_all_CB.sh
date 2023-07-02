make clean
make -j7 CFLAGS='-DEXE_TYPE=OUTORDER_EXE -DLANES_PER_CB=256'
mv ramulator ramulator_32SA_256LA
make clean
make -j7 CFLAGS='-DEXE_TYPE=OUTORDER_EXE -DLANES_PER_CB=512'
mv ramulator ramulator_32SA_512LA
make clean
make -j7 CFLAGS='-DEXE_TYPE=OUTORDER_EXE -DLANES_PER_CB=1024'
mv ramulator ramulator_32SA_1024LA
make clean
make -j7 CFLAGS='-DEXE_TYPE=OUTORDER_EXE -DLANES_PER_CB=2048'
mv ramulator ramulator_32SA_2048LA


make clean
make -j7 CFLAGS='-DDEBUG -DEXE_TYPE=OUTORDER_EXE -DLANES_PER_CB=256'
mv ramulator ramulator_32SA_256LA_verbose
make clean
make -j7 CFLAGS='-DDEBUG -DEXE_TYPE=OUTORDER_EXE -DLANES_PER_CB=512'
mv ramulator ramulator_32SA_512LA_verbose
make clean
make -j7 CFLAGS='-DDEBUG -DEXE_TYPE=OUTORDER_EXE -DLANES_PER_CB=1024'
mv ramulator ramulator_32SA_1024LA_verbose
make clean
make -j7 CFLAGS='-DDEBUG -DEXE_TYPE=OUTORDER_EXE -DLANES_PER_CB=2048'
mv ramulator ramulator_32SA_2048LA_verbose