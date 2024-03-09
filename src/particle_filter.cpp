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
#include <limits>

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
/**
 * @brief: data association for debug
 * @param: predicted, obstacles in robot coordinate
 * @param: observations in sensor, obstacles in robot coordinate
 * @Note: for different landmark form, how to association is the key
*/
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
    const int lenObservation = observations.size();
    const int lenPredictedLandMark = predicted.size();
    for(int m = 0; m < lenObservation; m++) {
        double minDis = std::numeric_limits<double> ::infinity();
        for(int n = 0; n < lenPredictedLandMark; n++) {
            auto& curObs = observations[m];
            auto curPre = predicted[n];
            auto curDis = dist(curObs.x, curObs.y, curPre.x, curPre.y);
            if(curDis < minDis) {
                minDis = curDis;
                curObs.id = curPre.id;
            }
        }
    }
}

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
// convert to to map system according to particles
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs>& observations,
                                   const Map& map_landmarks) {
    for(auto particle& : particles) {
        vector<LandmarkObs> obsInMap;
        for(auto& obs : obervations) {
            obsInMap.push_back(transLocal2Global(obs, particle));
        }
        vector<LandmarkObs> obsPredicted;
        for(auto& landmark : map_landmarks.landmark_list) {
            if(dist(landmark.x, landmark.y, particle.x, particle.y) <= sensor_range) {
                obsPredicted.push_back(landmark);
            }
        }
        dataAssociation(obsPredicted, obsInMap); // id filled
        // update association and weight
        particle.associations.clear();
        for(auto& obs : obsInMap) {
            particle.associations.push_back(obs.id);
            relatedLandmark = map_landmarks.landmark_list[obs.id];
            particle.weight *= calculate2dGuassian(std_landmark, relatedLandmark, obs)
        }
    }
}

/**
 * TODO: Resample particles with replacement with probability proportional
 *   to their weight.
 * NOTE: resample的本质在于根据概率随机舍弃
 */
void ParticleFilter::resample() {
    // get weights
    weights.clear();
    double sumWeight = 0;
    for(auto& particle : particles) {
        const double& curWeight = particle.weight;
        sumWeight += curWeight;
        weights.push_back(curWeight);
    }
    Eigen::VectorXd weightVec = Eigen::Map<Eigen::VectorXd>(weights.data(), weights.size());
    auto normalizedWeightVec = weightVec / sumWeight;
    double maxNormalizedWeight = normalizedWeightVec.maxCoeff();
    std::random_device rd; 
	// 使用 Mersenne Twister 引擎 
	std::mt19937 gen(rd()); 
	// 创建均匀分布器，范围为 [0, 1)
	std::uniform_real_distribution<> dis(0.0, 1.0); 
    // begin resample
    int index = 0;
    double beta = 0.0;
    std::vector sampledWeight;
    int lenStep = weights.size();
    for(int i = 0; i < lenStep; ++i) {
        beta += 2 * dis(gen) * maxNormalizedWeight;
        while(normalizedWeightVec(index) < beta) {
            beta -= normalizedWeightVec(index);
            index = (index + 1) % lenStep;
        }
        sampledWeight.push_back(particles[index]);
    }
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
