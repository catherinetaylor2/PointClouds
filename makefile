IDIR = -I/home/catherine/Documents/mc_dev/src/ -I/usr/include/pcl-1.8 -I/usr/include/vtk-6.3 -I/usr/include/boost -I/usr/include/pcl-1.8 -I/usr/include/
CC = clang++ -std=c++11
CFLAGS = $(IDIR) `pkg-config opencv --cflags`  `pkg-config eigen3 --cflags --libs`
ODIR = obj

LIBS = `pkg-config opencv --libs` -L/usr/local/lib/ -lGLEW -lglfw -lGL -L/usr/lib/x86_64-linux-gnu/ -lboost_filesystem -lboost_system -lboost_thread -lpcl_visualization -lpcl_io -lpcl_common -lpcl_kdtree -lvtkRenderingCore-6.3 -lvtkCommonDataModel-6.3 -lvtkCommonDataModel-6.3 -lvtkCommonMath-6.3 -lvtkCommonCore-6.3



_OBJ =  pointCloud.o
OBJ = $(patsubst %, $(ODIR)/%, $(_OBJ))

$(ODIR)/%.o: %.cpp ;
	$(CC) -c -o $@ $< $(CFLAGS) 


run: $(OBJ) ;
	clang++ -o $@ $^ $(CFLAGS) $(LIBS)

# .PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 

