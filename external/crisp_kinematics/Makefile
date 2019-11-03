CC = g++
CFLAGS = -g -std=gnu++11 -O3 -I/usr/include/eigen3 -I/playpen/alan/gsl-install/include -L/playpen/alan/gsl-install/lib/ -lgsl -lgslcblas

model:	main.o CosseratRod.o
	$(CC) $(CFLAGS) -o model main.o CosseratRod.o

main.o:	main.cpp CosseratRod.o Tools.h ParallelCosseratRod.h SystemSolver.h Integrator.h Quaternion.h CosseratRod.h ApplyToEntities.h ConstraintEntities.h DerivativeEntities.h Entities.h State.h RK8Integrator.h Observer.h
	$(CC) $(CFLAGS) -c main.cpp

CosseratRod.o: CosseratRod.cpp Tools.h ParallelCosseratRod.h SystemSolver.h Integrator.h Quaternion.h CosseratRod.h ApplyToEntities.h ConstraintEntities.h DerivativeEntities.h Entities.h State.h RK8Integrator.h Observer.h
	$(CC) $(CFLAGS) -c CosseratRod.cpp

clean:
	rm model *.o
