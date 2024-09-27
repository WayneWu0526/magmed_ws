# Makefile

# 指定编译器
CXX = g++

INCLUDES = -I/usr/include/eigen3 -I/usr/include

LIBDIRS = -L/usr/local/lib -L/usr/lib

LIBS = -lModernRoboticsCpp -lnlopt

# 指定编译选项
CXXFLAGS = -Wall -g $(INCLUDES)

LDFLAGS = $(LIBDIRS) $(LIBS)

# 指定目标文件
# TARGET = test_bvp
TARGET = test_optSolve

# 指定源文件
SRCS = test_optSolve.cpp

# 目标文件
OBJS = $(SRCS:.cpp=.o)

# 规则定义
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJS)
