test: test.o gizmobot/point.o
	$(LINK.cc) -o $@ $^

gizmobot/point.o:	gizmobot/point.cpp gizmobot/point.h

.PHONY:	clean
clean:
	rm -f test *.o gizmobot/*.o
