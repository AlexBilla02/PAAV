#ifndef TRACKER_H_
#define TRACKER_H_

#include "tracker/Tracklet.h"
#include <limits>
#include <unordered_map>
class Tracker
{
public:
  Tracker();
  ~Tracker();
  Tracker(double roi_x1, double roi_x2, double roi_y1, double roi_y2);

  int getROIEntryCount() const;

  // handle tracklets
  void removeTracks();
  void addTracks(const std::vector<bool> &associated_detections,
                 const std::vector<double> &centroids_x,
                 const std::vector<double> &centroids_y);

  // associate tracklets and detections
  void dataAssociation(std::vector<bool> &associated_detections,
                       const std::vector<double> &centroids_x,
                       const std::vector<double> &centroids_y);

  // track objects
  void track(const std::vector<double> &centroids_x,
             const std::vector<double> &centroids_y,
             bool lidarStatus);

  // getters
  const std::vector<Tracklet> &getTracks() { return tracks_; }

private:
  // tracklets
  std::vector<Tracklet> tracks_;
  int cur_id_;

  // association
  std::vector<std::pair<int, int>> associated_track_det_ids_;

  // thresholds
  double distance_threshold_;
  double covariance_threshold;
  int loss_threshold;

  int roi_entry_count_;  // Contatore del numero di oggetti entrati nella ROI
  double roi_x1;  // Limite inferiore della ROI in x
  double roi_x2;  // Limite superiore della ROI in x
  double roi_y1;  // Limite inferiore della ROI in y
  double roi_y2;  // Limite superiore della ROI in y
  std::unordered_map<int, bool> tracklet_in_roi_;  // Mappa per controllare se il tracklet è stato contato
};

#endif // TRACKER_H_
