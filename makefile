exe: main.o Dstar_lite_algorithm.o grid_input.o state.o execute.o
	g++ -o exe.out main.o Dstar_lite_algorithm.o grid_input.o state.o execute.o

main.o: main.cpp
	g++ -c main.cpp -std=c++11

Dstar_lite_algorithm.o: include/Dstar_lite_algorithm.cpp
	g++ -c include/Dstar_lite_algorithm.cpp -std=c++11

grid_input.o: include/grid_input.cpp
	g++ -c include/grid_input.cpp -std=c++11

state.o: include/state.cpp
	g++ -c include/state.cpp -std=c++11

execute.o: include/execute.cpp
	g++ -c include/execute.cpp -std=c++11

clean:
	rm -f *.o
