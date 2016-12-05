#ifndef LOADINGBAR_H
#define LOADINGBAR_H 1

#include <ctime>
#include <chrono>
#include <iostream>

#define LOADING_BAR_LENGTH 40

typedef std::chrono::system_clock::time_point sys_time_t;

class LoadingBar {
 public:
  void start(int total_iter, const char* message){
    c_start = std::clock();
    t_start = std::chrono::system_clock::now();
    std::cout << message; fflush(stdout);
    
    print_interval = total_iter / LOADING_BAR_LENGTH;
    iter = 0;
  }

  void load(){
    if (iter++ % print_interval == 0) {
      printf("-"); fflush(stdout);
    }
  }

  void end(){
    c_end = std::clock();
    t_end = std::chrono::system_clock::now();

    double cpu_time = (c_end-c_start) / (double) CLOCKS_PER_SEC;
    double wall_time = std::chrono::duration<double>(t_end-t_start).count();
    printf("] %7.3gs (%7.3gs CPU)\n",wall_time,cpu_time);
  }


 private:
  clock_t c_start;
  clock_t c_end;
  sys_time_t t_start;
  sys_time_t t_end;
  int iter;
  int print_interval;
};

#endif 