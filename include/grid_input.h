#ifndef PLANNER_GRID_INPUT_H
#define PLANNER_GRID_INPUT_H

#include <stdio.h>
#include <unistd.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

typedef int16_t ID_SIZE;
typedef std::pair<int16_t, int16_t> Points;

const int kFile_Numbers = 960;  //需要读取的文件数目
const char FilePath[] = "inputs//";

class GrideInput {
 public:
  GrideInput(int file_number) : file_num_(file_number) {}

  void GetOneGrid();  //得到一张地图
  void PrintMap();    //打印map

  inline int get_grid_rows() const { return rows_; }
  inline int get_grid_columns() const { return columns_; }
  inline Points get_start_pos() const { return start_pos_; }
  inline Points get_goal_pos() const { return goal_pos_; }
  inline std::map<ID_SIZE, Points> get_obstacle_pos() const {
    return obstacle_list_;
  }
  void PrintObstacle() const;

 private:
  int16_t rows_, columns_, file_num_;
  int16_t scene_flg_;
  Points start_pos_, goal_pos_;
  std::map<ID_SIZE, Points> obstacle_list_;

  inline ID_SIZE calculate_id(const Points& xoy) const {
    return static_cast<ID_SIZE>(xoy.first * columns_ + xoy.second);
  }
};

void GridInputOneMap(int num);                //获得一个文本的map
void AllGridInput(int16_t scene_select_num);  //输出全部的map

#endif