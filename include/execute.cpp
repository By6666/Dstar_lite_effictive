#include "execute.h"
std::vector<std::string> sum_result;  //存储总结输出结果的容器

/* 搜索一张map
 * 作用：执行一次完整的动态的D*lite，并进行结果统计
 * 输入：map序号
 * 输出：无
 * */
void SearchOneMap(int map_num) {
  /* 获得map信息 */
  GrideInput map_info(map_num);
  map_info.GetOneGrid();
  map_info.PrintMap();  //打印原始map

  /* 数据传入，构造D* lite算法对象 */
  Dstarlite Dstar_lite_algorithm(
      map_info.get_grid_rows(), map_info.get_grid_columns(),
      map_info.get_start_pos(), map_info.get_goal_pos(),
      map_info.get_obstacle_pos());

  while (1) {
    /* 规划路径 */
    std::cout << "**********" << std::endl;
    std::cout << "search num : " << Dstar_lite_algorithm.get_search_nums() + 1
              << std::endl;
    bool flg = Dstar_lite_algorithm.CalculatePath();

    /* 如果失败，结束循环 */
    if (!flg) {
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
      std::cout << "|final result : no path to goal !!|" << std::endl;
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
      Dstar_lite_algorithm.PrintCountResult();
      break;
    } else {
      Dstar_lite_algorithm.RecurPath();

      std::cout << std::endl << "search successful !!" << std::endl;
      Dstar_lite_algorithm.PrintSearchResult();
    }

    /* 沿着当前path移动 */
    while (!Dstar_lite_algorithm.get_current_path().empty()) {
      Dstar_lite_algorithm.UpdateObsNeighborsInfo();
      /* 如果下一个点是障碍物 */
      if (Dstar_lite_algorithm.NextStepIsInObstacleList())
        break;
      else
        Dstar_lite_algorithm.StartMove();
    }

    /* 走到了终点 */
    if (Dstar_lite_algorithm.ArriveGoal()) {
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
      std::cout << "|final result: get goal successflly!!|" << std::endl;
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
      Dstar_lite_algorithm.PrintCountResult();
      break;
    }

    Dstar_lite_algorithm.ResetStart();
  }

  /* 结果统计 */
  sum_result.push_back(
      std::to_string(map_num) + "          " +
      std::to_string(Dstar_lite_algorithm.get_search_nums()) + "          " +
      std::to_string(Dstar_lite_algorithm.get_all_expand_nums()) + " " +
      std::to_string(Dstar_lite_algorithm.get_move_step_nums()));
}

/* 打印统计结果
 * 输入：无
 * 输出：无
 *  */
void PrintSumResult() {
  std::cout << std::endl
            << "-——-——-——-——-——-——-——-——-***-——-——-——-——-——-——-——-——-——-"
            << std::endl
            << "-——                   Sum  Result                    ——-"
            << std::endl
            << "-——-——-——-——-——-——-——-——-***-——-——-——-——-——-——-——-——-——-"
            << std::endl;
  std::cout << "| map num | search nums | expand nums | move_step nums |"
            << std::endl;

  for (auto& elem : sum_result) {
    std::cout << elem << std::endl;
  }
}