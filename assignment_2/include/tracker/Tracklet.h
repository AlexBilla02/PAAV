#ifndef TRACKLET_H_
#define TRACKLET_H_

#include <vector>
#include <cmath>

#include "KalmanFilter.h"

class Tracklet
{
public:
  Tracklet(int idTrack, double x, double y);
  ~Tracklet();

  void enterROI(); // Metodo per gestire l'ingresso nella ROI
  void exitROI();  // Metodo per gestire l'uscita dalla ROI
  double getTimeInROI() const; // Restituisce il tempo totale nella ROI

  void predict();
  void update(double x, double y, bool lidarStatus);

  // getters
  double getX() { return kf_.getX(); }
  double getY() { return kf_.getY(); }
  double getXCovariance() { return kf_.getXCovariance(); }
  double getYCovariance() { return kf_.getYCovariance(); }
  int getLossCount() { return loss_count_; }
  int getId() const { return id_; }
  double getTotalDistance() const { return total_distance_; }
private:
  // filter
  KalmanFilter kf_;

  // tracklet id
  int id_;

  // number of loss since last update
  int loss_count_;
  double total_distance_;
  double time_in_roi_; // Tempo totale nella ROI
  bool in_roi_;        // Flag per controllare se il tracklet è nella ROI
  double entry_time_;  // Tempo di ingresso nella ROI
  bool has_entered_roi_;
};

#endif // TRACKLET_H_
