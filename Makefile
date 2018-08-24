test: test.o GizmoBot1/point.o
	$(LINK.cc) -o $@ $^

GizmoBot1/point.o:	GizmoBot1/point.cpp GizmoBot1/point.h

.PHONY:	clean
clean:
	rm -f test *.o GizmoBot1/*.o
