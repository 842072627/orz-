// sigint_stub.cpp — 为 python_interface_test 提供 sigint_handler 定义
#include <cstdlib>
void sigint_handler(int) { exit(1); }
