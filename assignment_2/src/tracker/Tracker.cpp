#include "tracker/Tracker.h"
#include <iostream>
Tracker::Tracker()
{
    cur_id_ = 0;
    distance_threshold_ = 1.5; // meters
    covariance_threshold = 1.0; 
    loss_threshold = 0; //number of frames the track has not been seen
}
Tracker::~Tracker()
{
}

/*
    This function removes tracks based on any strategy
*/
void Tracker::removeTracks()
{
    std::vector<Tracklet> tracks_to_keep;

    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        // TODO
        // Implement logic to discard old tracklets
        // logic_to_keep is a dummy placeholder to make the code compile and should be subsituted with the real condition
        bool logic_to_keep = true;
        if (logic_to_keep)
            tracks_to_keep.push_back(tracks_[i]);
    }

    tracks_.swap(tracks_to_keep);
}

/*
    This function add new tracks to the set of tracks ("tracks_" is the object that contains this)
*/
void Tracker::addTracks(const std::vector<bool> &associated_detections, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{
    // Adding not associated detections
    for (size_t i = 0; i < associated_detections.size(); ++i)
        if (!associated_detections[i])
            tracks_.push_back(Tracklet(cur_id_++, centroids_x[i], centroids_y[i]));
}

/*
    This function associates detections (centroids_x,centroids_y) with the tracks (tracks_)
    Input:
        associated_detection an empty vector to host the associated detection
        centroids_x & centroids_y measurements representing the detected objects
*/
void Tracker::dataAssociation(std::vector<bool> &associated_detections, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{

    //Remind this vector contains a pair of tracks and its corresponding
    associated_track_det_ids_.clear();
    for (size_t i = 0; i < tracks_.size(); ++i)
    {

        int closest_point_id = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t j = 0; j < associated_detections.size(); ++j)
        {
            // TODO
            // Implement logic to find the closest detection (centroids_x,centroids_y) 
            // to the current track (tracks_) 
            
        }

        // Associate the closest detection to a tracklet
        if (min_dist < distance_threshold_ && !associated_detections[closest_point_id])
        {
            associated_track_det_ids_.push_back(std::make_pair(closest_point_id, i));
            associated_detections[closest_point_id] = true;
        }
    }
}

void Tracker::track(const std::vector<double> &centroids_x,
                    const std::vector<double> &centroids_y,
                    bool lidarStatus)
{
    std::vector<bool> associated_detections(centroids_x.size(), false);
    // TODO: Predict the position
    //For each track --> Predict the position of the tracklets
    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        tracks_[i].predict();
        std::cerr << "Track " << i << " predizione: x = " << tracks_[i].getX()
                  << ", y = " << tracks_[i].getY() << "\n";
    }
    // TODO: Associate the predictions with the detections
    dataAssociation(associated_detections, centroids_x, centroids_y);
    for (const auto &assoc : associated_track_det_ids_)
    {
        auto det_id = assoc.first;
        auto track_id = assoc.second;
        std::cerr << "Associazione: rilevamento " << det_id
                  << " associato a traccia " << track_id << "\n";
    }
    // Update tracklets with the new detections
    for (int i = 0; i < associated_track_det_ids_.size(); ++i)
    {
        auto det_id = associated_track_det_ids_[i].first;
        auto track_id = associated_track_det_ids_[i].second;
        tracks_[track_id].update(centroids_x[det_id], centroids_y[det_id], lidarStatus);
        std::cerr << "Traccia " << track_id << " aggiornata a: x = " 
                  << tracks_[track_id].getX() << ", y = " << tracks_[track_id].getY() << "\n";
    }

    // TODO: Remove dead tracklets

    // TODO: Add new tracklets
}
