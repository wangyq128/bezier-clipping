#pragma once

#include <math.h>
#include <stdio.h>

//修改控制点后修改POW_P POW_Q
#define POW_P 4 //记录p的次数
#define POW_Q 7 //记录q的次数
#define MAXNUM 10 //记录交点最大个数
#define MAXCON 20 //记录控制点最大个数
#define FAIL -1 //表示没有交点

using namespace std;

typedef struct { double x; double y; } point;


