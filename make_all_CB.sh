make clean
make -j7 CFLAGS='-DEXETYPE=OUTORDER -DLANES_PER_SA=256'
mv ramulator ramulator_32SA_256LA
make clean
make -j7 CFLAGS='-DEXETYPE=OUTORDER -DLANES_PER_SA=512'
mv ramulator ramulator_32SA_512LA
make clean
make -j7 CFLAGS='-DEXETYPE=OUTORDER -DLANES_PER_SA=1024'
mv ramulator ramulator_32SA_1024LA
make clean
make -j7 CFLAGS='-DEXETYPE=OUTORDER -DLANES_PER_SA=2048'
mv ramulator ramulator_32SA_2048LA


make clean
make -j7 CFLAGS='-DDEBUG -DEXETYPE=OUTORDER -DLANES_PER_SA=256'
mv ramulator ramulator_32SA_256LA_verbose
make clean
make -j7 CFLAGS='-DDEBUG -DEXETYPE=OUTORDER -DLANES_PER_SA=512'
mv ramulator ramulator_32SA_512LA_verbose
make clean
make -j7 CFLAGS='-DDEBUG -DEXETYPE=OUTORDER -DLANES_PER_SA=1024'
mv ramulator ramulator_32SA_1024LA_verbose
make clean
make -j7 CFLAGS='-DDEBUG -DEXETYPE=OUTORDER -DLANES_PER_SA=2048'
mv ramulator ramulator_32SA_2048LA_verbose