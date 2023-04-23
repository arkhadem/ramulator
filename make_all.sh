make clean
make -j7 CFLAGS='-DEXE_TYPE=INORDER_EXE -DISA_TYPE=RISCV_ISA'
mv ramulator ramulator_riscv
make clean
make -j7 CFLAGS='-DEXE_TYPE=INORDER_EXE -DISA_TYPE=LIME_ISA'
mv ramulator ramulator_ino
make clean
make -j7 CFLAGS='-DEXE_TYPE=OUTORDER_EXE -DISA_TYPE=LIME_ISA'
mv ramulator ramulator_ooo
make clean
make -j7 CFLAGS='-DEXE_TYPE=DVI_EXE -DISA_TYPE=LIME_ISA'
mv ramulator ramulator_dvi
make clean
make -j7 CFLAGS='-DEXE_TYPE=ORACLE_EXE -DISA_TYPE=LIME_ISA'
mv ramulator ramulator_ora


make clean
make -j7 CFLAGS='-DDEBUG -DEXE_TYPE=INORDER_EXE -DISA_TYPE=RISCV_ISA'
mv ramulator ramulator_riscv_verbose
make clean
make -j7 CFLAGS='-DDEBUG -DEXE_TYPE=INORDER_EXE -DISA_TYPE=LIME_ISA'
mv ramulator ramulator_ino_verbose
make clean
make -j7 CFLAGS='-DDEBUG -DEXE_TYPE=OUTORDER_EXE -DISA_TYPE=LIME_ISA'
mv ramulator ramulator_ooo_verbose
make clean
make -j7 CFLAGS='-DDEBUG -DEXE_TYPE=DVI_EXE -DISA_TYPE=LIME_ISA'
mv ramulator ramulator_dvi_verbose
make clean
make -j7 CFLAGS='-DDEBUG -DEXE_TYPE=ORACLE_EXE -DISA_TYPE=LIME_ISA'
mv ramulator ramulator_ora_verbose

