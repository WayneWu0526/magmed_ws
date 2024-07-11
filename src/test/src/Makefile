# Makefile

# 指定编译器
CXX = g++

# 指定编译选项
CXXFLAGS = -I /usr/include/eigen3 -std=c++11

# 指定目标文件
TARGET = test_bvp

# 指定源文件
SRCS = test_bvp.cpp

# 指定对象文件
OBJS = $(SRCS:.cpp=.o)

# 编译目标文件
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

# 编译源文件
.cpp.o:
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 清理生成的文件
clean:
	rm -f $(OBJS) $(TARGET)
