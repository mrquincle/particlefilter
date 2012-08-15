/**
 * @brief 
 * @file PositionParticleFilter.cpp
 *
 * This file is created at Almende B.V. It is open-source software and part of the Common 
 * Hybrid Agent Platform (CHAP). A toolbox with a lot of open-source tools, ranging from 
 * thread pools and TCP/IP components to control architectures and learning algorithms. 
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless,
 * we personally strongly object to this software being used by the military, in factory 
 * farming, for animal experimentation, or anything that violates the Universal 
 * Declaration of Human Rights.
 *
 * Copyright © 2012 Anne van Rossum <anne@almende.com>
 *
 * @author  Anne C. van Rossum
 * @date    Aug 14, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
 */
#include <PositionParticleFilter.h>

using namespace dobots;

/* **************************************************************************************
 * Implementation of PositionParticleFilter
 * **************************************************************************************/

PositionParticleFilter::PositionParticleFilter() {
	bins = 16;
	seed = 234789;
	auto_coeff.clear();
	auto_coeff.push_back(2.0);
	auto_coeff.push_back(-1.0);
	srand48(seed);
}

PositionParticleFilter::~PositionParticleFilter() {

}

void PositionParticleFilter::Tick(CImg<DataValue> *img_frame)  {
	img = img_frame;
	Transition();
	Likelihood();
	Resample();
}

/**
 * Initialize the particle filter.
 */
void PositionParticleFilter::Init(NormalizedHistogramValues &tracked_object_histogram,
		CImg<int> &coord, int particle_count) {

	// generate duplicates of particles
	for (int i = 0; i < particle_count; ++i) {
		this->tracked_object_histogram = tracked_object_histogram;
		Particle<ParticleState> &p  = *new Particle<ParticleState>();
		ParticleState s = p.getState();
		s.width = coord(3) - coord(0);
		s.height = coord(4) - coord(1);
		s.x.push_back(coord(0) + s.width / 2);
		s.y.push_back(coord(1) + s.height / 2);
		//	s.histogram
		//	s.
		getParticles().push_back(p);
	}
}

/**
 * Transition of all particles following a certain motion model. Due to the fact there is
 * no multiplication or removal in the number of particles at this moment, we can safely
 * iterate through the entire container and do the transition "in place".
 */
void PositionParticleFilter::Transition() {
	std::vector<Particle<ParticleState> >::iterator i;
	for (i = getParticles().begin(); i != getParticles().end(); ++i) {
		doTransition(i->getState());
	}
}

void PositionParticleFilter::Likelihood() {
	std::vector<Particle<ParticleState> >::iterator i;
	for (i = getParticles().begin(); i != getParticles().end(); ++i) {
		i->setWeight(Likelihood(i->getState()));
	}
}

/**
 * Normal state of affairs is to use an autoregressive model to estimate where an
 * object will be next. There are however many different autoregressive models in use,
 * and it doesn't seem there is a "standard" way...:
 * <ul>
 * <li>Robust Visual Tracking for Multiple Targets (Cai, Freitas, Little):
 *     x[n] = A x[n-1] + B x[n-2] + C N(0,1) </li>
 * <li>CamShift Guided Particle Filter for Visual Tracking (Wang, Yang, Xy, Yu):
 *     x[n] = 2*x[n-1] - x[n-2] + N(0,1) </li>
 * <li>Real-time Hand Tracking using a Mean Shift Embedded Particle Filter (Shan,
 * Tan, Wei):
 *     (x[n] - x[n-1]) = (x[n-1] - x[n-2]) + N(0,1)</li>
 * <li>PFCHA: A New Moving Object Tracking Algorithms Based on Particle Filter
 * and Histogram (Qi, JiaFu):
 *     (x[n] - x_avg) = A (x[n-1] - x_avg) + B (x[n-2] - x_avg) + N(0,1)</li>
 * </ul>
 * Obviously, the second and third are the same. To have a second-order autoregressive
 * process "over differences" it is for example also possible to use:
 *   x[n+1] = x[n] + A (x[n] - x[n-1]) + B (x[n-1] - x[n-2]) + N(0,1)
 * etcetera, etcetera.
 * I think it's smart to consider a formulation where x[n], x[n-1], ... can be used
 * directly, so we do not need to store differences in our vector for "predict", hence
 * I opt for the second formulation (auto_coeff[0] = 2, auto_coeff[1] = -1).
 *
 */
ParticleState *PositionParticleFilter::Transition(ParticleState oldp) {
	ParticleState & newp = *new ParticleState();

	int xn = dobots::predict(oldp.x.begin(), oldp.x.end(), auto_coeff.begin(), 0.0);
	int yn = dobots::predict(oldp.y.begin(), oldp.y.end(), auto_coeff.begin(), 0.0);

	xn = std::max(0, std::min((int)img->_width-1, xn));
	yn = std::max(0, std::min((int)img->_height-1, yn));

	newp.height = oldp.height;
	newp.width = oldp.width;
	newp.histogram = oldp.histogram;

	// make sure here that also old values are copied to the new particle, not done yet
	assert(false);

	dobots::pushpop(newp.x.begin(), newp.x.end(), xn);
	dobots::pushpop(newp.y.begin(), newp.y.end(), yn);

	return &newp;
}

void PositionParticleFilter::doTransition(ParticleState oldp) {
	int xn = dobots::predict(oldp.x.begin(), oldp.x.end(), auto_coeff.begin(), 0.0);
	int yn = dobots::predict(oldp.y.begin(), oldp.y.end(), auto_coeff.begin(), 0.0);

	xn = std::max(0, std::min((int)img->_width-1, xn));
	yn = std::max(0, std::min((int)img->_height-1, yn));

	dobots::pushpop(oldp.x.begin(), oldp.x.end(), xn);
	dobots::pushpop(oldp.y.begin(), oldp.y.end(), yn);
}

/**
 * The likelihood of a player at all locations in the image using a given region size.
 */
void PositionParticleFilter::Likelihood(RegionSize region_size) {
	// for every particle calculate likelihood and "calculate" weight
	assert(false); // we don't calculate this (yet)
}

/**
 * Calculate the likelihood of a player and the state indicated by the parameter
 * "state" which contains an x and y position, a width and a height. This is used
 * to define a rectangle for which a histogram is matched against the reference
 * histogram of the object that is tracked.
 * @param state			the state of the particle (position, width, height)
 * @return				conceptual "distance" to the reference (tracked) object
 */
float PositionParticleFilter::Likelihood(ParticleState state) {
	CImg <unsigned char> img_selection = img->get_crop(
			state.x[0]-state.width/2,state.y[0]-state.height/2,state.x[0]+state.width/2,state.y[0]+state.height/2);
	DataFrames frames;
	pDataMatrix data = img_selection._data;
	frames.push_back(data);

	Histogram histogram(bins, img->_width, img->_height);
	cout << "Add data for histograms" << endl;
	histogram.calcProbabilities(frames);

	NormalizedHistogramValues result;
	histogram.getProbabilities(result);

	Value dist = dobots::distance<Value>(tracked_object_histogram, result, dobots::DM_HELLINGER);
	return dist;
}
