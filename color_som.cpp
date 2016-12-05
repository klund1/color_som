#include "SelfOrganizingMap.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <png++/png.hpp>

char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

int main(int argc, char * argv[]) {
  int som_size = 100;
  int color_depth = 2;
  int num_iter = 1000;
  int num_threads = 1;

  if (cmdOptionExists(argv,argv+argc, "-s")) {
    som_size = atoi(getCmdOption(argv,argv+argc,"-s"));
  }
  if (cmdOptionExists(argv,argv+argc, "-c")) {
    color_depth = atoi(getCmdOption(argv,argv+argc,"-c"));
  }
  if (cmdOptionExists(argv,argv+argc, "-n")) {
    num_iter = atoi(getCmdOption(argv,argv+argc,"-n"));
  }
  if (cmdOptionExists(argv,argv+argc, "-t")) {
    num_threads = atoi(getCmdOption(argv,argv+argc,"-t"));
  }

  float sigma = som_size/3;
  float final_sigma = sigma*0.22;
  float nu = 0.8;
  float final_nu = nu*0.22;

  SelfOrganizingMap som{som_size};
  som.setIter(num_iter);
  som.setThreads(num_threads);
  som.setSigma(sigma);
  som.setFinalSigma(final_sigma);
  som.setNu(nu);
  som.setFinalNu(final_nu);

  std::vector<point> colors;
  float color_spacing = 255.0/(color_depth-1);
  for(float r = 0.0; r < color_depth; ++r){
    for(float g = 0.0; g < color_depth; ++g){
      for(float b = 0.0; b < color_depth; ++b){
        point v(r*color_spacing, g*color_spacing, b*color_spacing);
        v.normalize();
        colors.push_back( v );
      }
    }
  }
  som.train(colors);

  png::image< png::rgb_pixel > imageA(som_size,som_size);
  for(int i = 0; i < som_size; ++i){
    for(int j = 0; j < som_size; j++){
      point weight = 255*som.node(i,j);
      imageA[i][j] = png::rgb_pixel(weight[0],weight[1],weight[2]);
    }
  }
  imageA.write("outA.png");

  point** winners = som.winners(colors);

  png::image< png::rgb_pixel > imageB(som_size,som_size);
  for(int i = 0; i < som_size; ++i){
    for(int j = 0; j < som_size; j++){
      point weight = 255*winners[i][j];
      imageB[i][j] = png::rgb_pixel(weight[0],weight[1],weight[2]);
    }
  }
  imageB.write("outB.png");


  for(int i = 0; i < som_size; ++i){
    delete[] winners[i];
  }
  delete[] winners;

  return 0;
}