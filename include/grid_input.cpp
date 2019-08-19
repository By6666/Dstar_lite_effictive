#include "grid_input.h"

void GrideInput::GetOneGrid() {
  std::string file_name = FilePath + std::to_string(file_num_) + ".txt";

  std::ifstream read_grid;
  read_grid.open(file_name, std::ios_base::in);
  if (!read_grid.is_open()) {
    std::cout << "file open failed !!" << std::endl;
    return;
  }

  read_grid >> rows_ >> columns_;

  std::string grid_buff = "";
  for (int16_t i = 0; i < rows_ + 1; ++i) {
    getline(read_grid, grid_buff);  //整行读取
    //获得起点、终点、障碍物坐标
    for (int16_t j = 0; j < grid_buff.length(); ++j) {
      if (grid_buff[j] == 's') {
        start_pos_.first = i - 1;
        start_pos_.second = j / 2;
      } else if (grid_buff[j] == 'g') {
        goal_pos_.first = i - 1;
        goal_pos_.second = j / 2;
      } else if (grid_buff[j] == 'x') {
        Points temp(i - 1, j / 2);
        obstacle_list_[calculate_id(temp)] = temp;
      }
    }
  }
  read_grid.close();
}

//打印一张地图
void GrideInput::PrintMap() {
  std::cout << "primal map :" << std::endl;
  std::cout << "row: " << rows_ << "   col: " << columns_ << std::endl;
  for (int i = 0; i < rows_; ++i) {
    for (int j = 0; j < columns_; ++j) {
      if (start_pos_ == Points(i, j))
        std::cout << "s ";
      else if (goal_pos_ == Points(i, j))
        std::cout << "g ";
      else if (obstacle_list_.find(calculate_id(Points(i, j))) !=
               obstacle_list_.end())
        std::cout << "x ";
      else
        std::cout << "_ ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl << std::endl;
}

void GrideInput::PrintObstacle() const {
  for (auto &iter : obstacle_list_) {
    std::cout << "ID : " << iter.first << "   | pos: (" << iter.second.first
              << "," << iter.second.second << ")" << std::endl;
  }
}

// for testing
void GridInputOneMap(int num) {  //获得一个文本的map
  GrideInput grid_info(num);
  grid_info.GetOneGrid();
  //输出map的行列数
  std::cout << "rows:" << grid_info.get_grid_rows()
            << " columns:" << grid_info.get_grid_columns() << std::endl;
  //输出起点与终点坐标
  std::cout << "start_pos:(" << grid_info.get_start_pos().first << ","
            << grid_info.get_start_pos().second << ")  goal_pos:("
            << grid_info.get_goal_pos().first << ","
            << grid_info.get_goal_pos().second << ")" << std::endl;
  grid_info.PrintMap();
  grid_info.PrintObstacle();
}

void AllGridInput(int16_t scene_select_num) {  //输出全部的map
  for (int i = 0; i < kFile_Numbers; ++i) {    //获得所有文本的map
    GridInputOneMap(i + 1);
    std::cout << std::endl;
  }
}
