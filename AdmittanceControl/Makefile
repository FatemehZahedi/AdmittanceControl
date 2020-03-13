BASE_DIR	= ../..
include $(BASE_DIR)/build/GNUMake/paths.mak
include $(BASE_DIR)/build/GNUMake/$(TOOLS_MAK)

# ---------------- HDF ----------------------
HDF_INSTALL = /usr/local/hdf5
HDF_LIB 	= $(HDF_INSTALL)/lib64
HDF_INCLUDE = -I$(HDF_INSTALL)/include
HDF_LIBS 	= -L$(HDF_LIB)	\
			  -lhdf5_cpp	\
			  -lhdf5 		\
			  -lz			\
			  -lm 			\
			  -ldl			\
			  #-lsz

# ------------------------------------------

SOURCES 		= AdmittanceControl.cpp \
			 	  PositionControlClient.cpp \
				  UdpServer.cpp

TARGET 		= AdmittanceControl

CXXFLAGS 		+=
LDFLAGS 		+= $(LIB_DIR)/libFRIClient.a
LDFLAGS			+= $(HDF_LIBS)
INCLUDE 		= 	-I/usr/include/	\
					$(HDF_INCLUDE)

include $(BASE_DIR)/build/GNUMake/rules_examples.mak
