#include "Dstar_lite_algorithm.h"

/* 构造函数
 * 作用：1、对类中成员变量进行赋值； 2、对算法进行初始化
 * 输入：map的 行、列、起点、终点、障碍物列表
 * 备注：没有传入引用，是为了让此类中有一份障碍物列表对象，防止外部对象的析构对该类造成影响
 * */
Dstarlite::Dstarlite(int16_t row, int16_t col, Points start, Points goal,
                     std::map<ID_SIZE, Points> obstacle_list)
    : row_(row),
      col_(col),
      start_pos_(start),
      goal_pos_(goal),
      all_obstacle_list_(obstacle_list) {
  /* initial */
  km_ = 0;
  move_step_nums_ = 0;
  search_nums_count_ = 0;
  all_expand_points_count_ = 0;
  last_start_ = curr_start_ = start_pos_;
  // current_obstacle_map_ = all_obstacle_list_;

  all_cell_info_map_[CalculateId(goal_pos_)] = CellInfo{
      goal_pos_,
      ValueInfo{CalculateId(goal), CalculateDistence(goal_pos_), 0, INF::max()},
      StateInfo{true, false}, -1};
  all_cell_info_map_[CalculateId(start_pos_)] = CellInfo{
      start_pos_, ValueInfo{CalculateId(start_pos_), 0, INF::max(), INF::max()},
      StateInfo{false, false}, -1};

  start_ = &all_cell_info_map_[CalculateId(start_pos_)];
  goal_ = &all_cell_info_map_[CalculateId(goal_pos_)];

  open_list_.push(CmpInfo{CalculateId(goal), CalculateKey(goal_->value_info)});
}

/* 计算最短路径
 * 作用：以当前起点为起点，进行一次路径规划，得出最短路径
 * 输入：无
 * 输出：ture：路径搜索成功； false：路径搜索失败
 * */
bool Dstarlite::CalculatePath() {
  current_expand_points_count_ = 0;
  ++search_nums_count_;

  /* 路径搜索循环 */
  while (open_list_.top().key < CalculateKey(start_->value_info) ||
         start_->value_info.rhs != start_->value_info.g) {
    /* 保证弹出的为in openlist状态 */
    if (!IsInOpenlist(open_list_.top().id)) {
      open_list_.pop();
      if (open_list_.empty()) break;
      continue;
    }

    /* 使用k_old缓存openlist中最小的元素，用于后续的比较 */
    Ty_KEY k_old = open_list_.top().key;

    /* 以下两行复杂度为logN (priority queue insert)*/
    CmpInfo curr_cell_cmp = open_list_.top();
    open_list_.pop();

    /* 以下一行复杂度为logN (map find)*/
    CellInfo* curr_cell_info = &all_cell_info_map_[curr_cell_cmp.id];

    /* 更新 h value */
    curr_cell_info->value_info.h = CalculateDistence(curr_cell_info->xoy);

    /* 更新curr_cell_cmp中的Key值 */
    curr_cell_cmp.key = CalculateKey(curr_cell_info->value_info);

    if (k_old < curr_cell_cmp.key) {
      /* 重新插入复杂度logN  (priority queue insert)*/
      open_list_.push(curr_cell_cmp);
    } else {
      /* 将cell的状态改为not in openlist */
      ChangeStateInOpenlist(curr_cell_info, StateValue::NOT);
      if (curr_cell_info->value_info.g > curr_cell_info->value_info.rhs) {
        curr_cell_info->value_info.g = curr_cell_info->value_info.rhs;
        UpdateVertex(curr_cell_info->xoy);
      } else {
        curr_cell_info->value_info.g = INF::max();
        UpdateVertexSingle(curr_cell_info->xoy);
        UpdateVertex(curr_cell_info->xoy);
      }
    }

    ++current_expand_points_count_;  //扩展点计数

    /* 如果openlist为空，则返回INF */
    if (open_list_.empty())
      open_list_.push(CmpInfo{-1, Ty_KEY(INF::max(), INF::max())});
  }

  all_expand_points_count_ += current_expand_points_count_;

  /* 当循环跳出时start点的g值仍为INF，说明找不到路径，返回false */
  if (start_->value_info.g == INF::max()) return false;

  return true;
}

/* 更新相邻点的状态
 * 作用：获得输入点的相邻点，并更新其状态
 * 输入：坐标点
 * 输出：无
 * 备注：类内private函数
 * */
void Dstarlite::UpdateVertex(const Points& pos) {
  /* 不会产生临时变量 */
  std::list<Points> neighbors = GetNeighbors(pos);

  for (auto& elem : neighbors) {
    UpdateVertexSingle(elem);
  }
}

/* 更新输入点的状态(single)
 * 作用：更新一个点的状态
 * 输入：需要更新的坐标点
 * 输出：无
 * 备注：类内private函数
 * */
void Dstarlite::UpdateVertexSingle(const Points& pos) {
  ID_SIZE pos_id = CalculateId(pos);

  /* meet goal --> return */
  if (pos_id == CalculateId(goal_pos_)) return;

  /* remove operator */
  if (!IsInInfoMap(pos_id)) {
    all_cell_info_map_[pos_id] =
        CellInfo{pos, ValueInfo{pos_id, INF::max(), INF::max(), INF::max()},
                 StateInfo{false, false}, -1};
  }
  CellInfo* cell = &all_cell_info_map_[pos_id];
  ChangeStateInOpenlist(cell, StateValue::NOT);

  /* get rhs */
  std::pair<VALUE_SIZE, ID_SIZE> temp = GetMinRhs(pos);
  cell->value_info.rhs = temp.first;
  cell->pre_best = temp.second;
  cell->value_info.h = CalculateDistence(pos);

  /* if g != rhs --> push */
  if (cell->value_info.g != cell->value_info.rhs) {
    open_list_.push(CmpInfo{pos_id, CalculateKey(cell->value_info)});
    ChangeStateInOpenlist(cell, StateValue::IN);
  }
}

/* 获得最小的rhs
 * 作用：找到该点最小的rhs
 * 输入：点坐标
 * 输出：最小的rhs，以及其对应的ID
 * 备注：类内private函数
 * */
std::pair<VALUE_SIZE, ID_SIZE> Dstarlite::GetMinRhs(const Points& pos) const {
  std::list<Points> neighbors = GetNeighbors(pos);
  VALUE_SIZE min_rhs = INF::max();
  ID_SIZE min_id = -1;

  if (neighbors.size() == 0) return std::make_pair(min_rhs, min_id);

  for (auto& elem : neighbors) {
    auto iter = all_cell_info_map_.find(CalculateId(elem));
    VALUE_SIZE temp = (iter == all_cell_info_map_.end())
                          ? INF::max()
                          : (iter->second.value_info.g + 1);

    temp = (temp == 0) ? INF::max() : temp;
    if (min_rhs > temp) {
      min_rhs = temp;
      min_id = CalculateId(elem);
    }
  }
  return std::make_pair(min_rhs, min_id);
}

/* 获取当前起点周围点的状态
 * 作用：获取当前start周围点的信息
 * 输入：无(使用类内current_start)
 * 输出：障碍物list
 * 备注：复杂度为 8*logN，为类内private
 * */
std::list<Points> Dstarlite::UpdateCurrStartNeighborsInfo() const {
  std::list<Points> neighbors;
  /* UP */
  if ((curr_start_.first - 1) >= 0) {
    Points temp(curr_start_.first - 1, curr_start_.second);
    if (IsNewObstacle(temp)) {  //确保障碍物是新的
      neighbors.push_back(temp);
    }
  }
  /* Down */
  if ((curr_start_.first + 1) < row_) {
    Points temp(curr_start_.first + 1, curr_start_.second);
    if (IsNewObstacle(temp)) {
      neighbors.push_back(temp);
    }
  }
  /* Left */
  if ((curr_start_.second - 1) >= 0) {
    Points temp(curr_start_.first, curr_start_.second - 1);
    if (IsNewObstacle(temp)) {
      neighbors.push_back(temp);
    }
  }
  /* Right */
  if ((curr_start_.second + 1) < col_) {
    Points temp(curr_start_.first, curr_start_.second + 1);
    if (IsNewObstacle(temp)) {
      neighbors.push_back(temp);
    }
  }
  return neighbors;
}

/* 更新新添加障碍物的临近点的信息
 * 作用：当得知一个障碍物时call，负责向当前obstacle list中添加
 * 输入：新的障碍物坐标
 * 输出：无
 * 备注：此函数为类内private
 * */
void Dstarlite::IncreaseObstacleInfo(const Points& obs) {
  all_cell_info_map_.erase(CalculateId(obs));
  current_obstacle_map_[CalculateId(obs)] = obs;

  std::list<Points> neighbors = GetNeighbors(obs);
  for (auto& elem : neighbors) {
    UpdateVertexSingle(elem);
  }
}

/* 更新获得障碍物周围的点
 * 作用：更新km，获得当前start点周围点的信息，并更新障碍物周围点的信息
 * 输入：无
 * 输出：无
 * 备注：调用3个类内private函数
 * */
void Dstarlite::UpdateObsNeighborsInfo() {
  km_ += DisOfLastToCurrStart();
  std::list<Points> neighbors = UpdateCurrStartNeighborsInfo();

  if (neighbors.empty()) return;

  for (auto& elem : neighbors) {
    IncreaseObstacleInfo(elem);
  }
}

/* 获得临近点的坐标值(四联通)
 * 作用：查看相邻的点是否可行，是否为障碍物
 * 输入：要查询的坐标点
 * 输出：所有临近点的list
 * 备注：复杂度为 4*logN，类内private函数
 * */
std::list<Points> Dstarlite::GetNeighbors(const Points& pos) const {
  std::list<Points> neighbors;
  /* UP */
  if ((pos.first - 1) >= 0 &&
      !IsObstacle(CalculateId(Points(pos.first - 1, pos.second)),
                  current_obstacle_map_)) {
    neighbors.push_back(Points(pos.first - 1, pos.second));
  }
  /* Down */
  if ((pos.first + 1) < row_ &&
      !IsObstacle(CalculateId(Points(pos.first + 1, pos.second)),
                  current_obstacle_map_)) {
    neighbors.push_back(Points(pos.first + 1, pos.second));
  }
  /* Left */
  if ((pos.second - 1) >= 0 &&
      !IsObstacle(CalculateId(Points(pos.first, pos.second - 1)),
                  current_obstacle_map_)) {
    neighbors.push_back(Points(pos.first, pos.second - 1));
  }
  /* Right */
  if ((pos.second + 1) < col_ &&
      !IsObstacle(CalculateId(Points(pos.first, pos.second + 1)),
                  current_obstacle_map_)) {
    neighbors.push_back(Points(pos.first, pos.second + 1));
  }
  return neighbors;
}

/* 路径回溯
 * 作用：回溯获得路径
 * 输入：无
 * 输出：无
 * */
void Dstarlite::RecurPath() {
  current_path_.clear();
  ID_SIZE temp_id = CalculateId(curr_start_);
  while (temp_id != -1) {
    auto iter = all_cell_info_map_[temp_id];
    current_path_.push_back(iter.xoy);
    temp_id = iter.pre_best;
  }
  current_path_.pop_front();
}

/* 打印一次搜索的结果
 * 作用：搜索结果的路径显示
 * 输入：无
 * 输出：无
 * */
void Dstarlite::PrintSearchResult() const {
  for (int i = 0; i < row_; ++i) {
    for (int j = 0; j < col_; ++j) {
      if (curr_start_ == Points(i, j))
        std::cout << "s ";

      else if (goal_pos_ == Points(i, j))
        std::cout << "g ";

      else if (IsObstacle(CalculateId(Points(i, j)), current_obstacle_map_))
        std::cout << "x ";

      else if (IsInPathList(Points(i, j)))
        std::cout << "o ";
      else
        std::cout << "_ ";
    }
    std::cout << std::endl;
  }
  std::cout << "shortest path step nums : " << current_path_.size()
            << "    expand point nums : " << current_expand_points_count_
            << std::endl;

  std::cout << std::endl << std::endl;
}

/* 打印一次搜索结果的路径点
 * 作用：显示路径上每一点的坐标值
 * 输入：无
 * 输出：无
 * */
void Dstarlite::PrintPath() const {
  std::cout << "------ Path ------" << std::endl;
  for (auto& elem : current_path_) {
    std::cout << "(" << elem.first << "," << elem.second << ")" << std::endl;
  }
}

/* 打印计数结果
 * 输入：无
 * 输出：无
 *  */
void Dstarlite::PrintCountResult() const {
  std::cout << std::endl
            << "The nums of search : " << search_nums_count_
            << "  total expanded nums : " << all_expand_points_count_
            << "   move steps : " << move_step_nums_ << std::endl
            << std::endl;
}
