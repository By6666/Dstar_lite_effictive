#include <time.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <vector>

#include "include/Dstar_lite_algorithm.h"
#include "include/execute.h"
#include "include/grid_input.h"

#define WHOLE 1

int main() {
#if WHOLE
  /* 总体运行 */
  clock_t start, end;
  start = clock();
  for (int i = 1; i <= kFile_Numbers; ++i) {
    std::cout << "/*******************************************/" << std::endl;
    std::cout << "/**********file——" << i << "**********/" << std::endl;
    SearchOneMap(i);
    std::cout << "/*******************************************/" << std::endl;
    std::cout << std::endl;
  }
  end = clock();

  /* 输出运行的时间 */
  printf("Spend time %.5f seconds!!\n", (float)(end - start) / CLOCKS_PER_SEC);
  std::cout << std::endl << std::endl;
  PrintSumResult();
#else
  int16_t map_num = 0;
  while (1) {
    std::cout << "Please input the map index (1~" << kFile_Numbers << "): ";
    std::cin >> map_num;
    if (map_num < 1 || map_num > kFile_Numbers) {
      std::cout << "Input wrong, Please input again !!" << std::endl;
      continue;
    }
    std::cout << "/*******************************************/" << std::endl;
    std::cout << "/**********file——" << map_num << "**********/" << std::endl;

    SearchOneMap(map_num);

    std::cout << "/*******************************************/" << std::endl;
    std::cout << std::endl;
    PrintSumResult();
  }
#endif
  return 0;
}
