/**
 * @Author: Erchao Rong
 * @Date:   2023-04-24 16:02:09
 * @Last Modified by:   Erchao Rong
 * @Last Modified time: 2023-04-24 22:45:48
 */
//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 17-Apr-2023 05:21:59
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include Files
#include "main.h"
#include "Zhang/Aerodynamics.h"
#include <iostream>
#include <fstream>  // 添加文件流支持
#include <cmath>    // 添加数学函数支持
// #include "Aerodynamics_terminate.h"
// #include "rt_nonfinite.h"
#include <iostream>
// Function Declarations
static void argInit_1x3_real_T(double result[3]);

static double argInit_real_T();

static void main_Aerodynamics();

// Function Definitions
//
// Arguments    : double result[3]
// Return Type  : void
//
static void argInit_1x3_real_T(double result[3])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 3; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_Aerodynamics()
{
    double F_b[3];
    double M_b[3];
    double dv[3] = {0};  // 初始化数组
    double alpha, beta;
    double V = 10.0;     // 速度大小
    
    // 打开文件准备写入数据
    std::ofstream outFile("alpha_vs_fz.txt");
    if (!outFile.is_open()) {
        std::cerr << "无法打开输出文件!" << std::endl;
        return;
    }

    // 设置alpha扫描范围和步长（覆盖完整的-π到π）
    const double start = -M_PI;
    const double end = M_PI;
    const double step = 0.05;
    
    // 添加M_PI定义（如果编译器未提供）
    #ifndef M_PI
    #define M_PI 3.14159265358979323846
    #endif

    for (double alpha_desired = start; alpha_desired <= end; alpha_desired += step) {
        // 使用极坐标表示法覆盖整个圆盘
        dv[0] = V * std::cos(alpha_desired);  // X分量
        dv[1] = 0.0;                         // Y分量（侧滑角=0）
        dv[2] = V * std::sin(alpha_desired);  // Z分量
        
        // 调用气动函数
        Aerodynamics(dv, F_b, M_b, &alpha, &beta);
        
        // 计算速度大小的平方（用于标准化）
        double V_sq = dv[0]*dv[0] + dv[1]*dv[1] + dv[2]*dv[2];
        
        // 写入数据：期望的alpha值和标准化后的F_b[2]
        outFile << alpha << "," << F_b[2]/V_sq << "\n";
    }

    outFile.close();
    std::cout << "数据已写入 alpha_vs_fz.txt" << std::endl;
}

//
// Arguments    : int argc
//                char **argv
// Return Type  : int
//
int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_Aerodynamics();
  // Terminate the application.
  // You do not need to do this more than one time.
  // Aerodynamics_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
