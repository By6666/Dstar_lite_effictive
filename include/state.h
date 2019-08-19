#ifndef PLANNER_STATE_H
#define PLANNER_STATE_H

#include <stdint.h>
#include <algorithm>
#include <utility>

#include "grid_input.h"

typedef bool STATE_SIZE;
typedef uint16_t VALUE_SIZE;
typedef std::pair<VALUE_SIZE, VALUE_SIZE> Ty_KEY;

/* 状态枚举 */
enum StateValue {
  NOT = false,
  IN = true
};

/* 一个cell所具有的值 */
struct ValueInfo {
  ID_SIZE id;
  VALUE_SIZE h;
  VALUE_SIZE rhs;
  VALUE_SIZE g;
};

/* openlist中存放的结构体 */
struct CmpInfo {
  ID_SIZE id;
  Ty_KEY key;

  bool operator<(const CmpInfo& rgh) const {
    if (key.first == rgh.key.first)
      return key.second > rgh.key.second;
    else
      return key.first > rgh.key.first;
  }
};

/* cell的状态信息 */
struct StateInfo {
  STATE_SIZE in_openlist;
  STATE_SIZE is_expand;
};

/* 一个cell具有的全部信息 */
struct CellInfo {
  Points xoy;
  ValueInfo value_info;
  StateInfo state_info;
  ID_SIZE pre_best;
};

bool operator<(const Ty_KEY& lf, const Ty_KEY& rh);


#endif
