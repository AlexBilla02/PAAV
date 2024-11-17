#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include "particle/particle_filter.h"
using namespace std;

static  default_random_engine gen;

/*
* TODO
* This function initialize randomly the particles
* Input:
*  std - noise that might be added to the position
*  nParticles - number of particles
*/
void ParticleFilter::init_random(double std[], int nParticles) {
  num_particles = nParticles;

  std::normal_distribution<double> dist_x(-std[0], std[0]); //random value between [-noise.x,+noise.x]
  std::normal_distribution<double> dist_y(-std[1], std[1]);
  std::normal_distribution<double> dist_theta(-std[2], std[2]);
//TODO
  for(int i=0; i < num_particles; i++){
    particles.push_back(Particle(dist_x(gen), dist_y(gen), dist_theta(gen))); 
  }
    is_initialized=true;

}

/*
* TODO
* This function initialize the particles using an initial guess
* Input:
*  x,y,theta - position and orientation
*  std - noise that might be added to the position
*  nParticles - number of particles
*/ 
void ParticleFilter::init(double x, double y, double theta, double std[],int nParticles) {
    num_particles = nParticles;
    normal_distribution<double> dist_x(-std[0], std[0]); //random value between [-noise.x,+noise.x]
    normal_distribution<double> dist_y(-std[1], std[1]);
    normal_distribution<double> dist_theta(-std[2], std[2]);

	//TODO
    for(int i=0; i < num_particles; i++){
        particles.push_back(Particle(x+dist_x(gen), y+dist_y(gen), theta+dist_theta(gen))); 
    }
    is_initialized=true;
}

/*
* TODO
* The predict phase uses the state estimate from the previous timestep to produce an estimate of the state at the current timestep
* Input:
*  delta_t  - time elapsed beetween measurements
*  std_pos  - noise that might be added to the position
*  velocity - velocity of the vehicle
*  yaw_rate - current orientation
* Output:
*  Updated x,y,theta position
*/
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    for(int i=0; i < num_particles; i++){
        double x,y,theta;
        double x_p = particles[i].x;
        double y_p = particles[i].y;
        double theta_p = particles[i].theta;
        if (std::fabs(yaw_rate) < 0.00001) {
            x = x_p+velocity*delta_t*std::cos(theta_p);
            y = y_p+velocity*delta_t*std::sin(theta_p);
            theta = theta_p;
        }else{  
            theta = theta_p+yaw_rate*delta_t;
            x = x_p+(velocity/yaw_rate)*(std::sin(theta)-std::sin(theta_p));
            y = y_p+(velocity/yaw_rate)*(std::cos(theta_p)-std::cos(theta));
        }   
    

        std::normal_distribution<double> dist_x(0, std_pos[0]);
        std::normal_distribution<double> dist_y(0, std_pos[1]);
        std::normal_distribution<double> dist_theta(0, std_pos[2]);

        particles[i].x = x + dist_x(gen);
        particles[i].y = y + dist_y(gen);
        particles[i].theta = theta + dist_theta(gen);
    }
}

/*
* TODO
* This function associates the landmarks from the MAP to the landmarks from the OBSERVATIONS
* Input:
*  mapLandmark   - landmarks of the map
*  observations  - observations of the car
* Output:
*  Associated observations to mapLandmarks (perform the association using the ids)
*/
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> mapLandmark, std::vector<LandmarkObs>& observations) {
   //TODO
   //TIP: Assign to observations[i].id the id of the landmark with the smallest euclidean distance
  for(int i=0; i<observations.size(); i++){
    double min_dist = std::numeric_limits<double>::max();
    int min_dist_id;
    for(int j=0; j<mapLandmark.size(); j++){
        double dist = std::sqrt((mapLandmark[j].x - observations[i].x) * 
                                (mapLandmark[j].x - observations[i].x) +
                                (mapLandmark[j].y - observations[i].y) *
                                (mapLandmark[j].y - observations[i].y));
        if(dist < min_dist){
            min_dist=dist;
            min_dist_id=mapLandmark[j].id;
        }
    }
    observations[i].id = min_dist_id;
  }
}

/*
* TODO
* This function transform a local (vehicle) observation into a global (map) coordinates
* Input:
*  observation   - A single landmark observation
*  p             - A single particle
* Output:
*  local         - transformation of the observation from local coordinates to global
*/
LandmarkObs transformation(LandmarkObs observation, Particle p){
    LandmarkObs global;
    
    global.id = observation.id;
    //global.x = -1; //TODO
    //global.y = -1; //TODO
    global.x = observation.x*std::cos(p.theta)-observation.y*std::sin(p.theta)+p.x; 
    global.y = observation.x*std::sin(p.theta)+observation.y*std::cos(p.theta)+p.y; 
    return global;
}

/*
* TODO
* This function updates the weights of each particle
* Input:
*  std_landmark   - Sensor noise
*  observations   - Sensor measurements
*  map_landmarks  - Map with the landmarks
* Output:
*  Updated particle's weight (particles[i].weight *= w)
*/
void ParticleFilter::updateWeights(double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {

    //Creates a vector that stores tha map (this part can be improved)
    std::vector<LandmarkObs> mapLandmark;
    for(int j=0;j<map_landmarks.landmark_list.size();j++){
        mapLandmark.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i,map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f});
    }
    for(int i=0;i<particles.size();i++){

        // Before applying the association we have to transform the observations in the global coordinates
        std::vector<LandmarkObs> transformed_observations;
        //TODO: for each observation transform it (transformation function)
        for(int j=0; j<observations.size();j++)
          transformed_observations.push_back(transformation(observations[j], particles[i]));        
        //TODO: perform the data association (associate the landmarks to the observations)
        dataAssociation(mapLandmark, transformed_observations);

        particles[i].weight = 1.0;
        // Compute the probability
		//The particles final weight can be represented as the product of each measurement’s Multivariate-Gaussian probability density
		//We compute basically the distance between the observed landmarks and the landmarks in range from the position of the particle
        for(int k=0;k<transformed_observations.size();k++){
            double obs_x,obs_y,l_x,l_y;
            obs_x = transformed_observations[k].x;
            obs_y = transformed_observations[k].y;
            //get the associated landmark 
            for (int p = 0; p < mapLandmark.size(); p++) {
                if (transformed_observations[k].id == mapLandmark[p].id) {
                    l_x = mapLandmark[p].x;
                    l_y = mapLandmark[p].y;
                }
            }	
			// How likely a set of landmarks measurements are, given a prediction state of the car 
            double w = std::exp( -( std::pow(l_x-obs_x,2)/(2*std::pow(std_landmark[0],2)) + std::pow(l_y-obs_y,2)/(2*std::pow(std_landmark[1],2)) ) ) / ( 2*M_PI*std_landmark[0]*std_landmark[1] );
            particles[i].weight *= w;
        }

    }    
}

/*
* TODO
* This function resamples the set of particles by repopulating the particles using the weight as metric
*/
void ParticleFilter::wheel_resample() {
    
    uniform_int_distribution<int> dist_distribution(0,num_particles-1);
    double beta  = 0.0;
    vector<double> weights;
    int index = dist_distribution(gen);
    vector<Particle> new_particles;

    for(int i=0;i<num_particles;i++)
        weights.push_back(particles[i].weight);
																
    float max_w = *max_element(weights.begin(), weights.end());
    uniform_real_distribution<double> uni_dist(0.0, max_w);
    for(int i=0; i < num_particles; i++){
        beta += uni_dist(gen)*2;
        while(weights[index] < beta){
          beta -= weights[index];
         index = (index + 1) % num_particles;
        }
        new_particles.push_back(particles[index]);
    }
    particles.swap(new_particles);

    //TODO write here the resampling technique (feel free to use the above variables)
}

std::vector<double> ParticleFilter::normalize(std::vector<double> weights){
  int N = weights.size();
  double s = 0.0;

  for(int i=0; i < N; i++){
    s+=weights[i];
  }

  for(int i=0; i < N; i++){
    weights[i] = weights[i]/s;
  }

  return weights;
}


void ParticleFilter::systematic_resample(){

  std::vector<double> weights;
  std::vector<Particle> new_particles;
  std::uniform_real_distribution<double> systematic_dist(0.0, 1.0/num_particles);

  for(int i=0;i<num_particles;i++)
    weights.push_back(particles[i].weight);
                              
  weights = normalize(weights);

  double c;
  c = weights[0];

  int i = 1;
  double rand = systematic_dist(gen);
  for(int j=0; j < weights.size(); j++){
    double u = rand + float(j) * 1.0/weights.size();
    while(u>c){
      i = (i + 1)%weights.size();
      c = c + weights[i];
    }
    new_particles.push_back(particles[i]);
  }
  particles.swap(new_particles);
}


