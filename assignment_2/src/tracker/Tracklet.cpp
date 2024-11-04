#include "tracker/Tracklet.h"
#include <iostream>
Tracklet::Tracklet(int idTrack, double x, double y)
{
  // set id
  id_ = idTrack;

  // initialize filter
  kf_.init(0.1);
  kf_.setState(x, y);

  // set loss count to 0
  loss_count_ = 0;
  total_distance_=0;
}

Tracklet::~Tracklet()
{
}

// Predict a single measurement
void Tracklet::predict()
{
  kf_.predict();
  loss_count_++;
}

// Update with a real measurement
void Tracklet::update(double x, double y, bool lidarStatus)
{
  Eigen::VectorXd raw_measurements_ = Eigen::VectorXd(2);

  // measurement update
  if (lidarStatus)
  {
    //posizione precedente
    double prev_x = kf_.getX();
    double prev_y = kf_.getY();
    std::cout<<"PREV:"<<prev_x<<" "<<prev_y<<"\n";
    // Aggiorno misura 
    raw_measurements_ << x, y;
    kf_.update(raw_measurements_);

    // Calcolo distanza percorsa dall'ultimo aggiornamento
    double dx = x - prev_x;
    double dy = y - prev_y;
    double distance_moved = std::sqrt(dx * dx + dy * dy);
    std::cout<<"DIFF:"<<distance_moved<<"\n";
    total_distance_ += distance_moved;
    std::cout<<"TOT_DIST:"<<total_distance_<<"\n";

    // Reset del contatore di perdita
    loss_count_ = 0;
  }
}
