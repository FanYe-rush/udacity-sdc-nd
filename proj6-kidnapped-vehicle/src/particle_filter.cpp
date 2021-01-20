/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    /**
     * TODO: Set the number of particles. Initialize all particles to
     *   first position (based on estimates of x, y, theta and their uncertainties
     *   from GPS) and all weights to 1.
     * TODO: Add random Gaussian noise to each particle.
     * NOTE: Consult particle_filter.h for more information about this method
     *   (and others in this file).
     */
    num_particles = 500;
    
    std::normal_distribution<float> x_distribution(x, std[0]);
    std::normal_distribution<float> y_distribution(y, std[1]);
    std::normal_distribution<float> t_distribution(theta, std[2]);
    
    std::default_random_engine generator;
    
    for (int i = 0; i < num_particles; i++) {
        Particle p;
        p.id = i;
        p.x = x_distribution(generator);
        p.y = y_distribution(generator);
        p.theta = t_distribution(generator);
        p.weight = 1.0f;
        particles.emplace_back(p);
        weights.emplace_back(1.0);
    }
    
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
    /**
     * TODO: Add measurements to each particle and add random Gaussian noise.
     * NOTE: When adding noise you may find std::normal_distribution
     *   and std::default_random_engine useful.
     *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
     *  http://www.cplusplus.com/reference/random/default_random_engine/
     */

    std::normal_distribution<float> x_noise(0, std_pos[0]);
    std::normal_distribution<float> y_noise(0, std_pos[1]);
    std::normal_distribution<float> theta_noise(0, std_pos[2]);
    
    std::default_random_engine generator;
    
    for (int i = 0; i < num_particles; i++) {
        Particle p = particles[i];
        
        double delta_theta = yaw_rate * delta_t;
        double final_theta = p.theta + delta_theta;
        if (yaw_rate > 0.0001) {
            particles[i].x += velocity / yaw_rate * (sin(final_theta) - sin(p.theta)) + x_noise(generator);
            particles[i].y += velocity / yaw_rate * (cos(p.theta) - cos(final_theta)) + y_noise(generator);
            particles[i].theta = final_theta + theta_noise(generator);
        } else {
            particles[i].x += velocity * delta_t * cos(p.theta) + x_noise(generator);
            particles[i].y += velocity * delta_t * sin(p.theta) + y_noise(generator);
            particles[i].theta = final_theta + theta_noise(generator);
        }
    }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
    /**
     * TODO: Find the predicted measurement that is closest to each
     *   observed measurement and assign the observed measurement to this
     *   particular landmark.
     * NOTE: this method will NOT be called by the grading code. But you will
     *   probably find it useful to implement this method and use it as a helper
     *   during the updateWeights phase.
     */
    for (int i = 0; i < observations.size(); i++) {
        LandmarkObs ob = observations[i];
        
        float min_dist = 100.0;
        float mx;
        float my;
        for (int j = 0; j < predicted.size(); j++) {
            LandmarkObs predict = predicted[j];
            float d = dist(predict.x, predict.y, ob.x, ob.y);
            if (d < min_dist) {
                min_dist = d;
                observations[i].id = predict.id;
                mx = predict.x;
                my = predict.y;
            }
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
    /**
     * TODO: Update the weights of each particle using a mult-variate Gaussian
     *   distribution. You can read more about this distribution here:
     *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
     * NOTE: The observations are given in the VEHICLE'S coordinate system.
     *   Your particles are located according to the MAP'S coordinate system.
     *   You will need to transform between the two systems. Keep in mind that
     *   this transformation requires both rotation AND translation (but no scaling).
     *   The following is a good resource for the theory:
     *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
     *   and the following is a good resource for the actual equation to implement
     *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
     */
    
    float var_x = std_landmark[0] * std_landmark[0];
    float var_y = std_landmark[1] * std_landmark[1];
    vector<LandmarkObs> predicted;
    vector<LandmarkObs> observations_copy(observations.size());
    
    double weight_total = 0.0;
    
    for (int i = 0; i < num_particles; i++) {
        Particle p = particles[i];
        
        predicted.clear();
        for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
            Map::single_landmark_s landmark = map_landmarks.landmark_list[j];
            if (dist(p.x, p.y, landmark.x_f, landmark.y_f) <= sensor_range) {
                LandmarkObs ob;
                ob.id = landmark.id_i;
                ob.x = landmark.x_f;
                ob.y = landmark.y_f;
                predicted.push_back(ob);
            }
        }
        
        for (int j = 0; j < observations.size(); j++) {
            LandmarkObs original = observations[j];
            observations_copy[j] = transform(original, p);
        }
        
        dataAssociation(predicted, observations_copy);
        
        vector<int> associations;
        
        double w = 1.0;
        for (int j = 0; j < observations_copy.size(); j++) {
            float x = observations_copy[j].x;
            float y = observations_copy[j].y;
            float mu_x = map_landmarks.landmark_list[observations_copy[j].id-1].x_f;
            float mu_y = map_landmarks.landmark_list[observations_copy[j].id-1].y_f;

            // calculate normalization term
            double gauss_norm = 1.0 / (2 * M_PI * std_landmark[0] * std_landmark[1]);

            // calculate exponent
            double exponent = (pow(x - mu_x, 2) / (2 * var_x)) + (pow(y - mu_y, 2) / (2 * var_y));
            w *= gauss_norm * exp(-exponent);
        }

        particles[i].weight = w;
        weight_total += w;
    }
    
    for (int i = 0; i < num_particles; i++) {
        weights[i] = particles[i].weight / weight_total;
    }
}

LandmarkObs ParticleFilter::transform(LandmarkObs ob, Particle p) {
    LandmarkObs transformed = LandmarkObs{ob.id, ob.x, ob.y};
    
    float sint = sin(p.theta);
    float cost = cos(p.theta);
    transformed.x = ob.x * cost - ob.y * sint + p.x;
    transformed.y = ob.x * sint + ob.y * cost + p.y;
    
    return transformed;
}

void ParticleFilter::resample() {
    /**
     * TODO: Resample particles with replacement with probability proportional
     *   to their weight.
     * NOTE: You may find std::discrete_distribution helpful here.
     *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
     */
    vector<Particle> new_samples(num_particles);
    
    std::discrete_distribution<> d(weights.begin(), weights.end());
    std::default_random_engine generator;
    
    for (int i = 0; i < num_particles; i++) {
        int r = d(generator);
        new_samples[i] = particles[r];
    }
    
    particles = new_samples;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
    // particle: the particle to which assign each listed association,
    //   and association's (x,y) world coordinates mapping
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
    vector<double> v;
    
    if (coord == "X") {
        v = best.sense_x;
    } else {
        v = best.sense_y;
    }
    
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
