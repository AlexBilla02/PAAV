#include "tracker/Tracklet.h"
#include <iostream>
#include <ctime>

double getCurrentTime() {
    return static_cast<double>(std::clock()) / CLOCKS_PER_SEC; // Tempo in secondi
}
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
  has_entered_roi_ = false; // campo per controllare se è mai entrato nella ROI

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
    // Aggiorno misura 
    raw_measurements_ << x, y;
    kf_.update(raw_measurements_);

    // Calcolo distanza percorsa dall'ultimo aggiornamento
    double dx = x - prev_x;
    double dy = y - prev_y;
    double distance_moved = std::sqrt(dx * dx + dy * dy);
    total_distance_ += distance_moved;

    // Reset del contatore di perdita
    loss_count_ = 0;
  }
}

void Tracklet::enterROI() {
    if (!in_roi_) {
        in_roi_ = true;
        entry_time_ = getCurrentTime(); 

        if (!has_entered_roi_) {
            time_in_roi_ = 0; 
            has_entered_roi_ = true; 
        }
    }
}

void Tracklet::exitROI() {
    if (in_roi_) {
        time_in_roi_ += getCurrentTime() - entry_time_;
        in_roi_ = false;
    }
}

double Tracklet::getTimeInROI() const {
  if (in_roi_){
    return time_in_roi_+(getCurrentTime()-entry_time_);
  }
  else
    return time_in_roi_;
}


