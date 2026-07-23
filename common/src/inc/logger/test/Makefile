all:
	g++ -I. -I../ test.cpp -o main -Wall -pthread --std=c++11
	
clean:
	rm -f *.o
	rm -f main

clean_logfile:
	rm -rf log

run:
	@./main && make --silent clean
