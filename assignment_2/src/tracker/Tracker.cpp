#include "tracker/Tracker.h"
#include <iostream>
Tracker::Tracker()
{
    cur_id_ = 0;
    distance_threshold_ = 0.5; // meters
    covariance_threshold = 0.0; 
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
    const size_t max_tracks = 10; // Limite massimo di tracce attive
    std::vector<Tracklet> tracks_to_keep;

    for (auto& track : tracks_)
    {
        if (track.getLossCount() < loss_threshold)
        {
            tracks_to_keep.push_back(track);
        }
    }

    // Assicurati che la lista di tracce non superi il numero massimo di tracce attive
    if (tracks_to_keep.size() > max_tracks) {
        tracks_to_keep.erase(tracks_to_keep.begin(), tracks_to_keep.end() - max_tracks);
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
            // Calcola la distanza euclidea tra il centro della rilevazione e la traccia corrente
            double dx = centroids_x[j] - tracks_[i].getX();
            double dy = centroids_y[j] - tracks_[i].getY();
            double dist = std::sqrt(dx * dx + dy * dy); // distanza euclidea
            
            // Se questa distanza è la minima trovata finora, aggiorna la distanza minima e l'indice del rilevamento
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_point_id = j;
            }
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
    removeTracks();
    std::vector<bool> associated_detections(centroids_x.size(), false);
    addTracks(associated_detections, centroids_x, centroids_y);
    // TODO: Predict the position
    //For each track --> Predict the position of the tracklets
    std::cerr<<tracks_.size();
    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        // Log della previsione
        std::cerr << "Prima della predizione - Track " << i
                  << " posizione attuale: x = " << tracks_[i].getX()
                  << ", y = " << tracks_[i].getY() << "\n";
                  
        tracks_[i].predict();
        
        // Log dopo la predizione
        std::cerr << "Dopo la predizione - Track " << i 
                  << " predizione: x = " << tracks_[i].getX()
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
        std::cerr << "Prima dell'aggiornamento - Track " << track_id 
                  << " predizione: x = " << tracks_[track_id].getX()
                  << ", y = " << tracks_[track_id].getY() << "\n";

        tracks_[track_id].update(centroids_x[det_id], centroids_y[det_id], lidarStatus);

        std::cerr << "Dopo l'aggiornamento - Track " << track_id 
                  << " nuova posizione: x = " << tracks_[track_id].getX()
                  << ", y = " << tracks_[track_id].getY() << "\n";
    }
    
    // TODO: Remove dead tracklets

    // TODO: Add new tracklets
}


