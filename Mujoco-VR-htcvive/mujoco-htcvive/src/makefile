COMMON=/O2 /MT /EHsc /arch:AVX /I../../mujoco200/include ../../mujoco200/bin/mujoco200.lib /Fe../../mujoco200/bin/  /I../../mujoco200/eigen/

all:
	cl $(COMMON) satyyr_main.cpp /I../src/include /I../src/lib ../../mujoco200/bin/glfw3.lib ../../mujoco200/bin/glew32.lib ../../mujoco200/bin/openvr_api.lib satyrr_controller.cpp potential_field.cpp /link OpenGL32.lib 
	del *.obj