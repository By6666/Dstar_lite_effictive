#ifndef PLANNER_DSTAR_LITE_ALGORITHM_H
#define PLANNER_DSTAR_LITE_ALGORITHM_H

#include <stdint.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <queue>
#include <vector>

#include "grid_input.h"
#include "state.h"

/* float最大值 */
typedef std::numeric_limits<VALUE_SIZE> INF;

class Dstarlite {
 public:
  /* 构造函数 */
  Dstarlite(int16_t row, int16_t col, Points start, Points goal,
            std::map<ID_SIZE, Points> obstacle_list);

  /* 析构函数 */
  ~Dstarlite() {}

  /* 路径回溯 */
  void RecurPath();

  /* 计算路径 */
  bool CalculatePath();

  /* 更新获得障碍物周围的点 */
  void UpdateObsNeighborsInfo();

  /* 沿着当前path移动一步 */
  void StartMove() {
    curr_start_ = current_path_.front();
    current_path_.pop_front();
    ++move_step_nums_;
  }

  /* 设置起点 */
  inline void ResetStart() {
    start_ = &all_cell_info_map_[CalculateId(curr_start_)];
  }

  /* 是否到达终点 */
  inline bool ArriveGoal() const { return curr_start_ == goal_pos_; }

  /* 判断下一步是否为obs */
  inline bool NextStepIsInObstacleList() const {
    return IsObstacle(CalculateId(current_path_.front()),
                      current_obstacle_map_);
  }

  /* 打印一次搜索的结果 */
  void PrintSearchResult() const;

  /* 打印一次 D* lite 搜索的结果 */
  void PrintCountResult() const;

  /* 打印路径 */
  void PrintPath() const;

  /* 获取类内成员 */
  inline const int32_t& get_move_step_nums() const { return move_step_nums_; }
  inline const int32_t& get_search_nums() const { return search_nums_count_; }
  inline const int32_t& get_all_expand_nums() const {
    return all_expand_points_count_;
  }
  inline const std::list<Points>& get_current_path() const {
    return current_path_;
  }

 private:
  VALUE_SIZE km_;
  int16_t row_, col_;
  Points start_pos_, goal_pos_;
  Points last_start_, curr_start_;
  std::list<Points> current_path_;

  CellInfo *start_, *goal_;

  std::map<ID_SIZE, Points> all_obstacle_list_;
  std::priority_queue<CmpInfo> open_list_;
  std::map<ID_SIZE, Points> current_obstacle_map_;
  mutable std::map<ID_SIZE, CellInfo> all_cell_info_map_;

  int32_t current_expand_points_count_,  //一次算法中的expand计数
      all_expand_points_count_,          //整体算法中的expand计数
      search_nums_count_,                //搜索次数计数
      move_step_nums_;                   //移动步数计数

  /* 计算key值 */
  inline Ty_KEY CalculateKey(const ValueInfo& value) const {
    VALUE_SIZE g_rhs_min = std::min(value.g, value.rhs);
    VALUE_SIZE key_1 =
        (g_rhs_min == INF::max()) ? INF::max() : (g_rhs_min + value.h + km_);
    return Ty_KEY(key_1, g_rhs_min);
  }

  /* 计算点的ID号 */
  inline ID_SIZE CalculateId(const Points& pos) const {
    return static_cast<ID_SIZE>(pos.first * col_ + pos.second);
  }

  /* 计算当前点与start点的曼哈顿距离 */
  inline VALUE_SIZE CalculateDistence(const Points& curr_pos) const {
    return static_cast<VALUE_SIZE>(abs(curr_pos.first - curr_start_.first) +
                                   abs(curr_pos.second - curr_start_.second));
  }

  /* 计算km */
  VALUE_SIZE DisOfLastToCurrStart() {
    VALUE_SIZE temp =
        static_cast<VALUE_SIZE>(abs(last_start_.first - curr_start_.first) +
                                abs(last_start_.second - curr_start_.second));
    last_start_ = curr_start_;
    return temp;
  }

  /* 获取当前起点周围点的状态 */
  std::list<Points> UpdateCurrStartNeighborsInfo() const;

  /* 添加障碍物信息 */
  void IncreaseObstacleInfo(const Points& pos);

  /* 更新输入点的状态 */
  void UpdateVertexSingle(const Points& pos);

  /* 更新相邻点的状态 */
  void UpdateVertex(const Points& pos);

  /* 获得所有临近点的坐标值 */
  std::list<Points> GetNeighbors(const Points& pos) const;

  /* 改变状态中in_openlist 0:NOT IN  1:IN*/
  inline void ChangeStateInOpenlist(CellInfo* cell, STATE_SIZE state_value) {
    cell->state_info.in_openlist = state_value;
  }

  /* 判断openlist中的top状态是否为In openlist */
  inline bool IsInOpenlist(ID_SIZE id) const {
    auto iter = all_cell_info_map_.find(id);
    if (iter == all_cell_info_map_.end())
      return false;
    else
      return iter->second.state_info.in_openlist;
  }

  /* 判断当前点是否为障碍物 */
  inline bool IsObstacle(ID_SIZE id,
                         const std::map<ID_SIZE, Points>& map) const {
    /* 以下一行复杂度为logN (map find)*/
    return map.find(id) != map.end();
  }

  /* 判断当前点是否在all_map中 */
  inline bool IsInInfoMap(ID_SIZE id) const {
    /* 以下一行复杂度为logN (map find)*/
    return all_cell_info_map_.find(id) != all_cell_info_map_.end();
  }

  /* 获得最小的rhs以及其对应ID */
  std::pair<VALUE_SIZE, ID_SIZE> GetMinRhs(const Points& pos) const;

  /* 判断点是否在pathlist中 */
  inline bool IsInPathList(const Points& pos) const {
    return std::find(current_path_.begin(), current_path_.end(), pos) !=
           current_path_.end();
  }

  /* 判断是否为新增障碍物(在全部障碍物map中，但不在当前障碍物map中) */
  inline bool IsNewObstacle(const Points& pos) const {
    ID_SIZE temp_id = CalculateId(pos);
    return IsObstacle(temp_id, all_obstacle_list_) &&
           !IsObstacle(temp_id, current_obstacle_map_);
  }
};

#endif