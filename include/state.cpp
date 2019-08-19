#include "state.h"


/* 为Ty_KEY重载小于号
 * 作用：用于calculatepath函数的主循环判断
 * 输入：小于号两边的比较元素
 * 输出：true or false
 * 备注：虽然这个操作可以使用函数来表示，但为了使代码更加明了，使用重载的小于号
 *  */
bool operator<(const Ty_KEY& lf, const Ty_KEY& rh){
  if (lf.first == rh.first)
    return lf.second < rh.second;
  else
    return lf.first < rh.first;
}
