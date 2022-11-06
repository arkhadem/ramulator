make clean
make -j7 CFLAGS='-DEXETYPE=INORDER'
mv ramulator ramulator_ino
make clean
make -j7 CFLAGS='-DEXETYPE=OUTORDER'
mv ramulator ramulator_ooo
make clean
make -j7 CFLAGS='-DEXETYPE=DVI'
mv ramulator ramulator_dvi
make clean
make -j7 CFLAGS='-DEXETYPE=ORACLE'
mv ramulator ramulator_ora


make clean
make -j7 CFLAGS='-DDEBUG -DEXETYPE=INORDER'
mv ramulator ramulator_ino_verbose
make clean
make -j7 CFLAGS='-DDEBUG -DEXETYPE=OUTORDER'
mv ramulator ramulator_ooo_verbose
make clean
make -j7 CFLAGS='-DDEBUG -DEXETYPE=DVI'
mv ramulator ramulator_dvi_verbose
make clean
make -j7 CFLAGS='-DDEBUG -DEXETYPE=ORACLE'
mv ramulator ramulator_ora_verbose

