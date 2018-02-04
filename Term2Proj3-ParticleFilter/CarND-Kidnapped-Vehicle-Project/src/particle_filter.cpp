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
	// GPS  x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Number of particles to draw
	num_particles = 10;

	default_random_engine gen;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for(int i=0; i<num_particles; i++){
		Particle p;
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1.0;
		particles.push_back(p);
		// cout<<"pid= "<<p.id<<" px= "<<p.x<<" py= "<<p.y<<" pt="<<p.theta<<endl;//////////////
	}

	is_initialized = true; 
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;
	normal_distribution<double> dist_x0(0, std_pos[0]);
	normal_distribution<double> dist_y0(0, std_pos[1]);
	normal_distribution<double> dist_theta0(0, std_pos[2]);

	if(fabs(yaw_rate)>0.001){
		for(int i=0; i<num_particles; i++){
			Particle p = particles[i];
			p.x += velocity/yaw_rate*(sin(p.theta+yaw_rate*delta_t)-sin(p.theta))+dist_x0(gen);
			p.y += velocity/yaw_rate*(cos(p.theta)-cos(p.theta+yaw_rate*delta_t))+dist_y0(gen);
			p.theta += yaw_rate*delta_t+dist_theta0(gen);
			particles[i] = p;
			// cout<<"Pred#1 pid= "<<p.id<<" px= "<<p.x<<" py= "<<p.y<<" pt="<<p.theta<<endl;//////////////
		}	
	}else{
		for(int i=0; i<num_particles; i++){
			Particle p = particles[i];
			p.x += velocity*cos(p.theta)*delta_t+dist_x0(gen);
			p.y += velocity*sin(p.theta)*delta_t+dist_y0(gen);
			p.theta += yaw_rate*delta_t+dist_theta0(gen);
			particles[i] = p;
			// cout<<"Pred#2 pid= "<<p.id<<" px= "<<p.x<<" py= "<<p.y<<" pt="<<p.theta<<endl;//////////////
		}
	}
	
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	for(int iOBS=0; iOBS<observations.size(); iOBS++){
		LandmarkObs obs = observations[iOBS];
		float min_dis;
		int min_dis_id;
		float dis;
		for(int iPre=0; iPre<predicted.size(); iPre++){
			LandmarkObs pre = predicted[iPre];
			dis = sqrt(pow(pre.x-obs.x,2) + pow(pre.y-obs.y,2));
			if (iPre==0){
				min_dis = dis;
				min_dis_id = pre.id;
			}else{
				if (dis<min_dis){
					min_dis = dis;
					min_dis_id = pre.id;	
				}
			}
		}
		//use observations id's to transfer matching id back to updateWeights function
		observations[iOBS].id = min_dis_id;

		// ///////////////////////////////////////////////////////////////////////////////////
		// for(int i=0; i<observations.size(); i++){///////////////////////////
		// 	LandmarkObs o = observations[i];
		// 	if (o.id == min_dis_id){
		// 		cout<<"Association: o id= "<<o.id<<" ox= "<<o.x<<" oy= "<<o.y<<endl;//////////////
		// 	}
		// }////////////////////////////////
		// for(int i=0; i<predicted.size(); i++){//////////////////////
		// 	LandmarkObs p = predicted[i];
		// 	if (p.id == min_dis_id){
		// 		cout<<"Association: p id= "<<p.id<<" px= "<<p.x<<" py= "<<p.y<<endl;//////////////
		// 	}
		// }///////////////////
		// ////////////////////////////////////////////////////////////////////////////////////////////
	}
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
	for(int iP=0; iP<num_particles; iP++){
		
		std::vector<LandmarkObs> predicted;
		Particle p = particles[iP];
		particles[iP].weight = 1;
		// cout<<"Each Particle pid= "<<p.id<<" px= "<<p.x<<" py= "<<p.y<<" pt="<<p.theta<<endl;//////////////

		//filter only landmarks in sensor_range
		for(int iLM=0; iLM<map_landmarks.landmark_list.size(); iLM++){
			float lm_x = map_landmarks.landmark_list[iLM].x_f;
			float lm_y = map_landmarks.landmark_list[iLM].y_f;
			int lm_id = map_landmarks.landmark_list[iLM].id_i;
			if(fabs(lm_x-p.x)<sensor_range && fabs(lm_y-p.y)<sensor_range){
				predicted.push_back(LandmarkObs{lm_id, lm_x, lm_y});
				// cout<<"LM in sensor_range id= "<<lm_id<<" x= "<<lm_x<<" y= "<<lm_y<<endl;//////////////
			}
		}
		//transform observation from car coordinate to map coordinate
		std::vector<LandmarkObs> obs_map_frame;
		for(int iOBS=0; iOBS<observations.size(); iOBS++){
			LandmarkObs obs = observations[iOBS];
			// cout<<"LM in car_coord id= "<<obs.id<<" x= "<<obs.x<<" y= "<<obs.y<<endl;//////////////
			double x = p.x + cos(p.theta)*obs.x - sin(p.theta)*obs.y;
			double y = p.y + sin(p.theta)*obs.x + cos(p.theta)*obs.y;
			obs_map_frame.push_back(LandmarkObs{iOBS, x, y});
			// cout<<"LM in map_coord id= "<<obs.id<<" x= "<<x<<" y= "<<y<<endl;//////////////
		}

		dataAssociation(predicted, obs_map_frame);//the closest prediction id is recorded in obs_map_frame id

		for(int iO=0; iO<obs_map_frame.size(); iO++){
			double o_x = obs_map_frame[iO].x;
			double o_y = obs_map_frame[iO].y;
			double p_x, p_y;

			for(int iPre=0; iPre<predicted.size(); iPre++){
				if(predicted[iPre].id == obs_map_frame[iO].id){
					 p_x = predicted[iPre].x;
					 p_y = predicted[iPre].y;
				}
			}
			// cout<<"weight Gaussian o_x= "<<o_x<<" o_y= "<<o_y<<" p_x= "<<p_x<<" p_y= "<<p_y<<endl;//////////////
			double obs_weight = exp(-pow(p_x-o_x,2)/2/pow(std_landmark[0],2)-pow(p_y-o_y,2)/2/pow(std_landmark[1],2));
			obs_weight /= 2.0*M_PI*std_landmark[0]*std_landmark[1];
			// cout<<"weight result = "<<obs_weight<<endl;//////////////////
			particles[iP].weight *= obs_weight;

		}

	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::vector<Particle> new_particles;
	new_particles.resize(particles.size());

	double sum_weight = 0;
	for (int j=0; j<particles.size(); j++){
		sum_weight += particles[j].weight;
	}
	// cout << "weight sum = " << sum_weight << endl;////////////////////

	std::default_random_engine gen;
  	std::uniform_real_distribution<double> rand_weight(0.0,sum_weight);

  	int index = 0;
  	float beta = 0;

	for(int iPar=0; iPar<particles.size(); iPar++){
		beta += rand_weight(gen);
		while(beta>particles[index].weight){
			beta -= particles[index].weight;
			index = (index+1)%particles.size();
		}
		new_particles[iPar] = particles[index];
		// cout << "new_particles weight= "<<new_particles[iPar].weight<<endl;
	} 
	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
