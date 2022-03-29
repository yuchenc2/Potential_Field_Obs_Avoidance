COMMON=/O2 /MT /EHsc /arch:AVX /I../include /Fe../bin/

all:
	cl $(COMMON) satyrr_potential.cpp ../bin/glfw3.lib ../bin/mujoco200.lib
	del *.obj
