###############################################################
# Purpose: Makefile for "sprint_two_colors"
# Author.: Bryan Wodi
# Version: 0.1
# License: GPL
# Date:    Aug 14, 2016
###############################################################

TARGET = sprint

INCLUDE_DIRS = -I../../include -I../../../Framework/include -I/usr/local/include/opencv -I/usr/local/include
# remember to take out debug and g options when finished testing
CXX = g++ -DDEBUG -g
CXXFLAGS += -O2 -DLINUX -Wall $(INCLUDE_DIRS)
LFLAGS += -lpthread -ljpeg -lrt -L/usr/local/lib -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_ocl -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab -lrt -lpthread -lm -ldl

OBJECTS = VisionMode.o StatusCheck.o main.o

all: $(TARGET)

# debug: CXXFLAGS += -DDEBUG -g
# debug: CXX += -DDEBUG -g
# debug: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

libclean:
	make -C ../../build clean

distclean: clean libclean

darwin.a:
	make -C ../../build

$(TARGET): darwin.a $(OBJECTS)
	$(CXX) $(CFLAGS) $(OBJECTS) ../../lib/darwin.a -o $(TARGET) $(LFLAGS)
	chmod 755 $(TARGET)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/DARwIn_sprint_two_colors_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
