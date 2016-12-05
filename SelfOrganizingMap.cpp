#include "SelfOrganizingMap.hpp"

SelfOrganizingMap::SelfOrganizingMap(int size) : size_{size}, 
                                                 num_iter_{1000},
                                                 threads_{1},
                                                 nu_{0.8},
                                                 final_nu_{0.1},
                                                 sigma_{((float) size)/3},
                                                 final_sigma_{((float) size)/10}
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dist(0, 255);


  colormap_ = new point*[size];
  for(int i = 0; i < size_; ++i){
    colormap_[i] = new point[size];
  }

  //initialize the bias to 1 for every node
  bias_ = new float*[size];
  for(int i = 0; i < size_; ++i){
    bias_[i] = new float[size];
    for(int j = 0; j < size_; ++j){
      bias_[i][j] = 1.0;
    }
  }

  loading_bar_.start(size*size_,"Constructing SOM:           [");
  for(int i = 0; i < size_; ++i){
    for(int j = 0; j < size_; ++j){
      colormap_[i][j] = point(dist(gen),dist(gen),dist(gen));
      colormap_[i][j].normalize();
      loading_bar_.load();
    }
  }
  loading_bar_.end();
}

SelfOrganizingMap::~SelfOrganizingMap(){
  for(int i = 0; i < size_; ++i){
    delete[] colormap_[i];
  }
  delete[] colormap_;

  for(int i = 0; i < size_; ++i){
    delete[] bias_[i];
  }
  delete[] bias_;
}

point& SelfOrganizingMap::node(int i, int j){
  return colormap_[i][j];
}


node_index SelfOrganizingMap::closest_node(point const& input){
  node_index winner;
  float best_corr = -1e10;

  if (threads_ == 1){
    for(int i = 0; i < size_; ++i){
      for(int j = 0; j < size_; j++){
        float corr = input.dot(node(i,j)) + bias_[i][j];
        if (corr > best_corr) {
          best_corr = corr;
          winner[0] = i;
          winner[1] = j;
        }
      }
    }
  }
  else {
    std::thread* pool = new std::thread[threads_];
    std::atomic_int pt_num{0};
    std::mutex mu;
    for(int thread_num = 0; thread_num < threads_; ++thread_num){
      pool[thread_num] = std::thread([this, &mu, &input, &winner, &best_corr, &pt_num]
        {
          node_index my_winner;
          float my_best_corr = -1e10;

          while (true) {
            int my_pt_num = pt_num++;
            int i = my_pt_num / size_;
            int j = my_pt_num % size_;

            if (i >= size_) {
              break;
            }

            float corr = input.dot(node(i,j)) + bias_[i][j];
            if (corr > my_best_corr) {
              my_best_corr = corr;
              my_winner[0] = i;
              my_winner[1] = j;
            }
          }
          mu.lock();
          if (my_best_corr > best_corr) {
              best_corr = my_best_corr;
              winner = my_winner;
          }
          mu.unlock();
        });
    }
    for(int thread_num = 0; thread_num < threads_; ++thread_num){
      pool[thread_num].join();
    }
    delete[] pool;
  }
  return winner;
}

void SelfOrganizingMap::train_one_sample(point const& input){

  //find the winner
  node_index winner = closest_node(input);
  int winner_i = winner[0];
  int winner_j = winner[1];
  bias_[winner_i][winner_j] = bias_[winner_i][winner_j] * 0.99;

  //update the weights
  if (threads_==1){
    for(int i = 0; i < size_; ++i){
      for(int j = 0; j < size_; j++){
        float dist_sq = pow(i-winner_i,2) + pow(j-winner_j,2); 
        float learning_factor = nu_ * exp(-dist_sq/(2*pow(sigma_,2)));
        node(i,j) = node(i,j) + learning_factor * (input - node(i,j));
        node(i,j).normalize();
      }
    }
  }
  else {
    std::thread* pool = new std::thread[threads_];
    std::atomic_int pt_num{0};
    for(int thread_num = 0; thread_num < threads_; ++thread_num){
      pool[thread_num] = std::thread([this, &input, winner_i, winner_j, &pt_num]
        {
          while (true) {
            int my_pt_num = pt_num++;
            int i = my_pt_num / size_;
            int j = my_pt_num % size_;

            if (i >= size_) {
              break;
            }
            float dist_sq = pow(i-winner_i,2) + pow(j-winner_j,2); 
            float learning_factor = nu_ * exp(-dist_sq/(2*pow(sigma_,2)));
            node(i,j) = node(i,j) + learning_factor * (input - node(i,j));
            node(i,j).normalize();
          }
        });
    }
    for(int thread_num = 0; thread_num < threads_; ++thread_num){
      pool[thread_num].join();
    }
    delete[] pool;
  }
}

void SelfOrganizingMap::train(std::vector<point> const& inputs){
  loading_bar_.start(num_iter_,"Training:                   [");

  int num_pts = inputs.size();

  //initialize random number generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dist(0,num_pts-1);

  float gamma = pow(final_nu_ / nu_, 1.0/num_iter_);
  float d_sigma = pow(final_sigma_ / sigma_, 1.0/num_iter_);
  
  for (int i = 0; i < num_iter_; ++i) {
    int node_index = dist(gen);
    train_one_sample(inputs[node_index]);
    nu_ = gamma * nu_;
    sigma_ = d_sigma * sigma_ ;
    loading_bar_.load();
  }
  loading_bar_.end();

}


point** SelfOrganizingMap::winners(std::vector<point> const& inputs){

  int num_pts = inputs.size();

  point** winner_map = new point*[size_];
  for(int i = 0; i < size_; ++i){
    winner_map[i] = new point[size_];
  }

  for(int i = 0; i < size_; ++i){
    for(int j = 0; j < size_; j++){
      winner_map[i][j] = point(0,0,0);
    }
  }


  loading_bar_.start(size_*size_,"Constructing rtree and map: ["); 
  bgi::rtree< rtree_point, bgi::rstar<16> > rtree;
  std::unordered_map<rtree_point,node_index,rtree_point_hash,rtree_point_eq> indices;
  for(int i = 0; i < size_; ++i){
    for(int j = 0; j < size_; j++){
      point& pt = node(i,j);
      float x = pt[0];
      float y = pt[1];
      float z = pt[2];
      rtree.insert(rtree_point(x,y,z));
      indices[rtree_point(x,y,z)] = node_index(i,j);
      loading_bar_.load();
    }
  }
  loading_bar_.end();

  

  loading_bar_.start(num_pts,"Finding Winners:            [");
  if (threads_ == 1){
    int pt_num = 0;
    std::vector<rtree_point> latest_winner;
    for(auto& pt : inputs) {

      rtree_point rpt{pt[0],pt[1],pt[2]};
      rtree.query(bgi::nearest(rpt,1),std::back_inserter(latest_winner));
      node_index winner_index = indices.at(latest_winner[0]);
      latest_winner.pop_back();

      int i = winner_index[0];
      int j = winner_index[1];
      winner_map[i][j] = pt;

      loading_bar_.load();

      ++pt_num;
    }
  }
  else {
    std::thread* pool = new std::thread[threads_];
    std::atomic_int pt_num{0};
    std::mutex mu;
    for(int thread_num = 0; thread_num < threads_; ++thread_num){
      pool[thread_num] = std::thread([this, &mu, winner_map, &inputs, &pt_num, 
                                      &rtree, &indices, num_pts]{
        std::vector<rtree_point> latest_winner;
        while (true) {
          int my_pt_num = pt_num++;

          if (my_pt_num >= num_pts) {
            break;
          }

          const point& pt = inputs[my_pt_num];
          rtree_point rpt{pt[0],pt[1],pt[2]};
          rtree.query(bgi::nearest(rpt,1),std::back_inserter(latest_winner));
          node_index winner_index = indices.at(latest_winner[0]);
          latest_winner.pop_back();

          int i = winner_index[0];
          int j = winner_index[1];
            
          mu.lock();
            winner_map[i][j] = pt;
            loading_bar_.load();
          mu.unlock();
        }
      });
    }
    for(int thread_num = 0; thread_num < threads_; ++thread_num){
      pool[thread_num].join();
    }
    delete[] pool;
  }
  loading_bar_.end();

  return winner_map;
}

void SelfOrganizingMap::setIter(int iter) {
  num_iter_ = iter;
}

void SelfOrganizingMap::setThreads(int threads) {
  threads_ = threads;
}

void SelfOrganizingMap::setNu(float nu){
  nu_ = nu;
}

void SelfOrganizingMap::setFinalNu(float final_nu){
  final_nu_ = final_nu;
}

void SelfOrganizingMap::setSigma(float sigma){
  sigma_ = sigma;
}

void SelfOrganizingMap::setFinalSigma(float final_sigma){
  final_sigma_ = final_sigma;
}

size_t rtree_point_hash::operator()(const rtree_point &pt) const{
  size_t h1 = std::hash<float>()(pt.get<0>());
  size_t h2 = std::hash<float>()(pt.get<1>());
  size_t h3 = std::hash<float>()(pt.get<2>());
  return (h1 ^ (h2 << 1)) ^ h3;
}

bool rtree_point_eq::operator()(const rtree_point &p1,const rtree_point &p2) const{
  return p1.get<0>()==p2.get<0>() && 
         p1.get<1>()==p2.get<1>() &&
         p1.get<2>()==p2.get<2>();
}


