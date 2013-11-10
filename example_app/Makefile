
#tell the linker the rpath so that we don't have to muck with LD_LIBRARY_PATH, etc
dramsim_test: dramsim_test.cpp
	$(CXX) -g -o dramsim_test dramsim_test.cpp -I../include -L../lib -ldramsim -Wl,-rpath=../lib

clean:
	rm dramsim_test
