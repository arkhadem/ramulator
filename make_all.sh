make clean
make -j7 CFLAGS='-DNUM_V0_PIPELINES=1 -DNUM_V1_PIPELINES=1'
mv ramulator ramulator_1V

make clean
make -j7 CFLAGS='-DNUM_V0_PIPELINES=2 -DNUM_V1_PIPELINES=2'
mv ramulator ramulator_2V

make clean
make -j7 CFLAGS='-DDEBUG -DNUM_V0_PIPELINES=1 -DNUM_V1_PIPELINES=1'
mv ramulator ramulator_1V_debug

make clean
make -j7 CFLAGS='-DDEBUG -DNUM_V0_PIPELINES=2 -DNUM_V1_PIPELINES=2'
mv ramulator ramulator_2V_debug