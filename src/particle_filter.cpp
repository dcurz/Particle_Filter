/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	//Set the number of particles
	num_particles = 5000;

	//Will need this generator for addition of Gaussian noise in particle initialization
	default_random_engine gen;

	//Create distributions around provided measurement locales to sample from
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	//Initialize all particles to first position
	for(int i = 0; i < num_particles; i++){
		
		Particle a_Particle; 
		
		a_Particle.id = i;
		a_Particle.x = dist_x(gen);
		a_Particle.y = dist_y(gen);
		a_Particle.theta = dist_theta(gen);
		a_Particle.weight = 1.0;

		particles.push_back(a_Particle);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	//Will need this generator for addition of Gaussian noise in particle initialization
	default_random_engine gen;

	//Create distributions around 0 to add to calculated 
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	//run each prticle through motion model
	for(int i = 0; i < num_particles; i++){
		//unpack for readability
		double dt = delta_t;
		double v = velocity;
		double theta_d = yaw_rate;
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;

		//prevent divide by 0 issues
		if(theta_d < .001){theta_d = .001;}

		//motion model including addition of random measurement noise
		particles[i].x = x + (v/theta_d)*(sin(theta + (theta_d*dt))-sin(theta)) + dist_x(gen);
		particles[i].y = y + (v/theta_d)*(cos(theta)-cos(theta + (theta_d*dt))) + dist_y(gen);
		particles[i].theta =  theta + (theta_d*dt) + dist_theta(gen);
	}
}

int ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, LandmarkObs& observation) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	// Run through transformed actual observations and assign to closest predicted observation (aka map landmark)
	// Do something clever with a for loop where you just keep track of lowest distance (call that helper function)
	// and the id associated with it - and update if you get a better one.
	double currMinDist = 50.0;
	int minDistIndex = predicted.size() + 1; 

	//for the given observation check it against every landmark in the map. "Predicted" is just the vector of map landmarks
	for (int j = 0; j<predicted.size(); j++){
		
		double theDist = dist(predicted[j].x, predicted[j].y, observation.x, observation.y);
		
		if (theDist<currMinDist){
			currMinDist = theDist;
			minDistIndex = j; 
		}
	}
	//return the index of the closest map measurement 
	return minDistIndex;	
	}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html


	//1. Come up with a set of predicted observations for each particle (in the MAP coordinates) - wait, isn't this just the landmarks straight up? 

//for loop for  2,3,4
	//2. Transform actual observations into MAP coordinates (based on each individual particle's position? :( )
	//***SHould the Theta be negative . . . ???
	
	//cycle through every particle
	for(int i = 0; i < num_particles; i++){

		//reset particle weight to one since we use a *= to update it later
		particles[i].weight = 1; 
		
		//unpack particle locations for readability	
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;

		//create vector of observations to store translated observations
		std::vector<LandmarkObs> mapCoordObs(observations.size());

		//cycle through every observation  
		for(int j = 0; j < mapCoordObs.size(); j++){
			//transform from car (particle) coords to map coords **MAY STILL NEED NEGATED
			mapCoordObs[j].x = observations[j].x*cos(theta) - observations[j].y*sin(theta) + x; 
			mapCoordObs[j].y = observations[j].x*sin(theta) + observations[j].y*cos(theta) + y;
			//perform nearest neighbor association
			mapCoordObs[j].id = dataAssociation(map_landmarks, mapCoordObs[j]);
		
			//rename for clarity
			double sig_x = std_landmark[0];
			double sig_y = std_landmark[1]; 
			int k = mapCoordObs[j].id; 
			double x_obs = mapCoordObs[j].x;
			double y_obs = mapCoordObs[j].y;
			double mu_x = map_landmarks.landmark_list[k].x_f;
			double mu_y = map_landmarks.landmark_list[k].y_f;

			//use each individual observation to update the particle's weight
			double gauss_norm = (1/(2*M_PI*sig_x*sig_y));
			double exponent = ((x_obs - mu_x)*(x_obs - mu_x))/(2*(sig_x*sig_x))+((y_obs - mu_y)*(y_obs - mu_y))/(2*(sig_y*sig_y));
		
			double thisObsWeight = gauss_norm * exp(-exponent);

			particles[i].weight *= thisObsWeight;
		}
	}

	//Normalize weight of every particle
	double weightsSum = 0.0; 

	for(int k = 0; k<particles.size(); k++){
		weightsSum+=particles[k].weight;
	}	

	for(int m = 0; m<particles.size(); m++){
		particles[m].weight /= weightsSum;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	//Create list to hold weights
	list<double> allTheWeightsList; 

	//populte list with weights
	for (int n = 0; n<particles.size(); n++){
		allTheWeightsList.push_back(particles[n].weight);
	}

	//resample according to weights
	random_device rd; 
	mt19937 gen(rd());
	discrete_distribution<> d(allTheWeightsList);

	for (int p = 0; p<particles.size(); p++){
		int resampleID = d(gen);
		particles[p].x = particles[resampleID].x;
		particles[p].y = particles[resampleID].y;
		particles[p].theta = particles[resampleID].theta;
	}

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
