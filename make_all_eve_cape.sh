make clean
make -j7
mv ramulator ramulator_dc

make clean
make -j7 CFLAGS='-DDEBUG'
mv ramulator ramulator_dc_debug

make clean
make -j7 CFLAGS='-DEXE_TYPE=DVI_EXE -DISA_TYPE=LIME_ISA -DLANES_PER_CB=1024 -DLANES_PER_SA=256'
mv ramulator ramulator_gpic

make clean
make -j7 CFLAGS='-DEXE_TYPE=INORDER_EXE -DISA_TYPE=LIME_ISA -DLANES_PER_CB=1024 -DLANES_PER_SA=256'
mv ramulator ramulator_eve

make clean
make -j7 CFLAGS='-DEXE_TYPE=INORDER_EXE -DISA_TYPE=LIME_ISA -DLANES_PER_CB=256 -DLANES_PER_SA=64 -DLATENCY_FILE_NAME=\"eve_intrinsics_latency\"'
mv ramulator ramulator_eve4

make clean
make -j7 CFLAGS='-DEXE_TYPE=INORDER_EXE -DISA_TYPE=LIME_ISA -DLANES_PER_CB=256 -DLANES_PER_SA=64 -DLATENCY_FILE_NAME=\"cape_intrinsics_latency\"'
mv ramulator ramulator_cape

make clean
make -j7 CFLAGS='-DDEBUG -DEXE_TYPE=DVI_EXE -DISA_TYPE=LIME_ISA -DLANES_PER_CB=1024 -DLANES_PER_SA=256'
mv ramulator ramulator_gpic_debug

make clean
make -j7 CFLAGS='-DDEBUG -DEXE_TYPE=INORDER_EXE -DISA_TYPE=LIME_ISA -DLANES_PER_CB=1024 -DLANES_PER_SA=256'
mv ramulator ramulator_eve_debug

make clean
make -j7 CFLAGS='-DDEBUG -DEXE_TYPE=INORDER_EXE -DISA_TYPE=LIME_ISA -DLANES_PER_CB=256 -DLANES_PER_SA=64 -DLATENCY_FILE_NAME=\"eve_intrinsics_latency\"'
mv ramulator ramulator_eve4_debug

make clean
make -j7 CFLAGS='-DDEBUG -DEXE_TYPE=INORDER_EXE -DISA_TYPE=LIME_ISA -DLANES_PER_CB=256 -DLANES_PER_SA=64 -DLATENCY_FILE_NAME=\"cape_intrinsics_latency\"'
mv ramulator ramulator_cape_debug