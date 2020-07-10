#Author: Hongtao Zhang

.DEFAULT_GOAL:=default

### colored echo
RED='\033[0;31m'
GREEN='\033[0;32m'
NO_COLOR='\033[0m' 

### designate directories/folders
BUILD = build
PROTO = proto
PROTOBUF = protobuf-auto-gen
TARGET = vfirm.exe


### source and include path
CPP_INC = -Iinc
CPP_SRC = $(wildcard *.cpp) $(wildcard src/*.cpp)
PROTO_INC = -I$(PROTOBUF) 
PROTO_SRC = $(wildcard $(PROTOBUF)/*pb.cc) 
PROTODEF_SRC = $(wildcard $(PROTO)/*.proto)


### compiler and compiling flags
#COMPILER = g++
COMPILER = clang++ -Qunused-arguments
CPP_STANDARD = -std=c++17
BOOST_LIB = -lboost_system -lboost_thread -lpthread -lboost_chrono
SCI_MATH_LIB = -larmadillo
PROTO_LIB = -L/usr/local/lib -lprotobuf

### concatenate all flags
FLAGS =  $(CPP_STANDARD) $(CPP_INC) $(PROTO_INC) $(BOOST_LIB)
FLAGS += $(PROTO_LIB) $(SCI_MATH_LIB)
FLAGS += -g 



### make rule for object file 
##### proto objs are added after finishing compiling .proto files
OBJS = $(addprefix $(BUILD)/, $(notdir $(CPP_SRC:.cpp=.o)) )
PROTO_OBJS = $(addprefix $(PROTOBUF)/, $(notdir $(PROTODEF_SRC:.proto=.pb.o)) )


### add source to vpath list
vpath %.cpp $(dir $(CPP_SRC))
vpath %.pb.cc $(dir $(PROTO_SRC))


##### pipe '|'  :  on the right of the pipe are order-only prequisites  
### create build folder to store object files for faster future re-builds
$(PROTOBUF):
	mkdir $@

$(BUILD):
	mkdir $@


### build the dependency file for tracking header files
DEP_FLAG = -MMD -MP -MF"$(@:%.o=%.d)"


### make object files
$(PROTOBUF)/%.o: %.cc 
	$(COMPILER) -c $< $(FLAGS) -o $@

$(BUILD)/%.o: %.cpp | $(BUILD)
	$(COMPILER) -c $< $(FLAGS) $(DEP_FLAG) -o $@ 


### generate & compile google protobuf libraries
$(PROTOBUF)/proto_generated.makemsg: $(PROTODEF_SRC) | $(PROTOBUF)
	protoc -I=$(PROTO) --cpp_out=$(PROTOBUF) $(PROTO)/*.proto
	@touch $(PROTOBUF)/proto_generated.makemsg
	@echo ${GREEN}[proto-generate completed!]${NO_COLOR}          

$(PROTOBUF)/proto_compiled.makemsg: $(PROTOBUF) $(PROTOBUF)/proto_generated.makemsg
	@make $(PROTO_OBJS)
	@touch $(PROTOBUF)/proto_compiled.makemsg
	@echo ${GREEN}[proto-compile completed!]${NO_COLOR}  

protobuf: 
	@make -j --no-print-directory $(PROTOBUF)/proto_compiled.makemsg


### build targets
$(TARGET): $(OBJS) $(PROTO_OBJS) | $(BUILD) 
	@$(COMPILER) $(OBJS) $(PROTO_OBJS) $(FLAGS) -o $@ 
	@echo ${GREEN}[build finished successfully! $(TARGET) generated!] ${NO_COLOR}

### include the dependency file to track .hpp files' updates
-include $(wildcard $(BUILD)/*.d)

default:
	@make --no-print-directory protobuf
	@make -j --no-print-directory $(TARGET) 
	

clean:
	rm -rf $(BUILD)
	rm -rf $(PROTOBUF)
	rm -f *.exe
	rm -f *.d

clean-build:
	rm -rf $(BUILD)
	rm -f *.exe
	rm -f *.d


clean-proto:
	rm -rf $(PROTOBUF)

.PHONY: clean clean-build clean-proto protobuf

### stand alone build cmd example template for similar task:
# g++ xxx.cpp -Ixxx_inc xxx_src/*.cc xxx_src/*.cpp 
# -lboost_system -lboost_thread -lpthread -L/usr/local/lib -lprotobuf 
# -o xxx.exe 