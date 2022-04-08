COMMON=/O2 /MT /EHsc /arch:AVX /I../include /Fe../bin/

all:
	cl $(COMMON) satyrr_main.cpp ../bin/glfw3.lib ../bin/mujoco200.lib satyrr_controller.cpp potential_field.cpp
	del *.obj
