// sigint_stub.cpp — 为 python_interface_test 提供 sigint_handler 定义
// driver.cpp 有 main() 被 test 目标排除，所以单独提供这个 stub
#include <cstdlib>
void sigint_handler(int) { exit(1); }
