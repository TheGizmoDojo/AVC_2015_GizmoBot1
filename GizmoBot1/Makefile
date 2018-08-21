test: test.o point.o
	$(LINK.cc) -o $@ $^

point.o:	point.cpp point.h

.PHONY:	clean
clean:
	rm test *.o
