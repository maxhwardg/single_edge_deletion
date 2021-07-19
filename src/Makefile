COMPILER=g++
FLAGS=-O2

main: main.cpp faster.o
	$(COMPILER) -o main main.cpp faster.o $(FLAGS)

naive.o: naive.cpp naive.hpp graph_utils.o
	$(COMPILER) -c naive.cpp graph_utils.o $(FLAGS)

faster.o: faster.cpp faster.hpp graph_utils.o
	$(COMPILER) -c faster.cpp graph_utils.o $(FLAGS)

graph_utils.o: graph_utils.hpp graph_utils.cpp
	$(COMPILER) -c graph_utils.cpp $(FLAGS)

clean:
	rm *.o
