make clean
make -j7 CFLAGS='-DEXE_TYPE=INORDER_EXE -DISA_TYPE=LIME_ISA -DLANES_PER_CB=1024 -DLANES_PER_SA=256'
mv ramulator ramulator_eve

make clean
make -j7 CFLAGS='-DEXE_TYPE=INORDER_EXE -DISA_TYPE=LIME_ISA -DLANES_PER_CB=2048 -DLANES_PER_SA=64 -DLATENCY_FILE_NAME="cape_intrinsics_latency"'
mv ramulator ramulator_cape

make clean
make -j7 CFLAGS='-DDEBUG -DEXE_TYPE=INORDER_EXE -DISA_TYPE=LIME_ISA -DLANES_PER_CB=1024 -DLANES_PER_SA=256'
mv ramulator ramulator_eve_debug

make clean
make -j7 CFLAGS='-DDEBUG -DEXE_TYPE=INORDER_EXE -DISA_TYPE=LIME_ISA -DLANES_PER_CB=2048 -DLANES_PER_SA=64 -DLATENCY_FILE_NAME="cape_intrinsics_latency"'
mv ramulator ramulator_cape_debug