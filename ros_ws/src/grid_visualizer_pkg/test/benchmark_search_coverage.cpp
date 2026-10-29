// Manual microbenchmark; no machine-dependent pass/fail threshold.
#include <chrono>
#include <iostream>
#include "grid_visualizer_pkg/search_coverage.hpp"

using namespace grid_visualizer_pkg;

int main()
{
  SearchCoverage coverage(-20,-20,40,40,.5);
  coverage.observe({{-8,-8},{8,-8},{8,8},{-8,8}});
  auto blocked=[](SearchPoint p) {return distance(p,{3,0})<2 || distance(p,{3,2})<2;};
  double checksum=0;
  auto start=std::chrono::steady_clock::now();
  for (int n=0;n<30;++n) {
    auto views=coverage.views({n*.01,0},{{0,2}},blocked,2,10,2.09,12,{},true);
    for (auto view:views) {checksum+=view.score;}
  }
  auto middle=std::chrono::steady_clock::now();
  for (int n=0;n<100;++n) {
    for (int k=0;k<40;++k) {checksum+=coverage.gain({10,10},k*.157,2,10,2.09);}
  }
  auto end=std::chrono::steady_clock::now();
  std::cout << "search update: "
    << std::chrono::duration<double,std::milli>(middle-start).count()/30 << " ms\n"
    << "40 camera headings: "
    << std::chrono::duration<double,std::milli>(end-middle).count()/100 << " ms\n"
    << "checksum: " << checksum << '\n';
}
