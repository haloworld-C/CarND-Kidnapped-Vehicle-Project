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
     *   first position (based on estimates of x, y, theta and their
     * uncertainties from GPS) and all weights to 1.
     * TODO: Add random Gaussian noise to each particle.
     */
    num_particles = 100;  // TODO: Set the number of particles
    size_t stdByteLen = sizeof(std);
    if(stdByteLen == 0 || (stdByteLen / sizeof(std[0]) != 3)) {
        throw std::runtime_error("std size not equal to state dimension !!");
    }
    std::default_random_engine generator;
    std::normal_distribution<double> x_distribution(x, std[0]);
    std::normal_distribution<double> y_distribution(y, std[1]);
    std::normal_distribution<double> theta_distribution(theta, std[2]);
    for (size_t i = 0; i < num_particles; i++) {
        Particle particle;
        particle.id = i;
        particle.x = x_distribution(generator);
        particle.y = y_distribution(generator);
        particle.theta = theta_distribution(generator);
        particle.weight = 1.0;

        particles.push_back(particle);
        weights.push_back(1.0);
    }
    is_initialized = true;
}
// update step
/**
 * TODO: Add measurements to each particle and add random Gaussian noise.
 * NOTE: When adding noise you may find std::normal_distribution
 *   and std::default_random_engine useful.
 * NOTE: in reality std_pos is related to sensors 
 */
void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
    for (size_t i = 0; i < num_particles; i++) {
        auto& particle = particles[i];
        double curYaw = particle.theta;
        particle.x = particle.x + velocity * cos(curYaw) * delta_t;
        particle.y = particle.y + velocity * sin(curYaw) * delta_t;
        particle.theta = curYaw + yaw_rate * delta_t;
        setUncertain(particle, std_pos);
    }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
    /**
     * TODO: Find the predicted measurement that is closest to each
     *   observed measurement and assign the observed measurement to this
     *   particular landmark.
     */
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs>& observations,
                                   const Map& map_landmarks) {
    /**
     * TODO: Update the weights of each particle using a mult-variate Gaussian
     *   distribution. You can read more about this distribution here:
     *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
     * NOTE: The observations are given in the VEHICLE'S coordinate system.
     *   Your particles are located according to the MAP'S coordinate system.
     *   You will need to transform between the two systems. Keep in mind that
     *   this transformation requires both rotation AND translation (but no
     * scaling). The following is a good resource for the theory:
     *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
     *   and the following is a good resource for the actual equation to
     * implement (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
     */
}

void ParticleFilter::resample() {
    /**
     * TODO: Resample particles with replacement with probability proportional
     *   to their weight.
     * NOTE: You may find std::discrete_distribution helpful here.
     *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
     */
}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
    // particle: the particle to which assign each listed association,
    //   and association's (x,y) world coordinates mapping
    // associations: The landmark id that goes along with each listed
    // association sense_x: the associations x mapping already converted to
    // world coordinates sense_y: the associations y mapping already converted
    // to world coordinates
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

void 
ParticleFilter::setUncertain(Particle& particle, double std[]) {
    std::default_random_engine generator;
    std::normal_distribution<double> x_distribution(particle.x, std[0]);
    std::normal_distribution<double> y_distribution(particle.y, std[1]);
    std::normal_distribution<double> yaw_distribution(particle.theta, std[2]);
    // add uncertian noise
    particle.x = x_distribution(generator);
    particle.y = y_distribution(generator);
    particle.theta = yaw_distribution(generator);
}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
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
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}
