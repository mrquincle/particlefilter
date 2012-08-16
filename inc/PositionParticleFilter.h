/**
 * @brief 
 * @file PositionParticleFilter.hpp
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


#ifndef POSITIONPARTICLEFILTER_HPP_
#define POSITIONPARTICLEFILTER_HPP_

#include <ParticleFilter.hpp>
#include <CImg.h>

#include <Histogram.h>
#include <Container.hpp>
#include <Autoregression.hpp>

#include <cassert>

using namespace cimg_library;
using namespace std;

//! An unsigned char would not be enough for a resolution greater than 256x256
typedef int CoordValue;

struct RegionSize {
	int width;
	int height;
};

struct ParticleState {
	int width;
	int height;
	int histogram;

	std::vector<Value> x;
	std::vector<Value> y;

	friend std::ostream& operator<<(std::ostream& os, const ParticleState & ps) {
		os << '[' << ps.width << ',' << ps.height << ']';
		return os;
	}
};

/* **************************************************************************************
 * Interface of PositionParticleFilter
 * **************************************************************************************/

/**
 * The particle filter that is used for tracking a 2D screen "position" plus some
 * additional state "elaborations", such as width, height, and histogram.
 */
class PositionParticleFilter: public ParticleFilter<ParticleState> {
public:
	//! Default constructor
	PositionParticleFilter();

	//! Default destructor
	~PositionParticleFilter();

	/**
	 * Set image and calculate everything necessary... Provide multiple times the same frame
	 * if that is required.
	 */
	void Tick(CImg<DataValue> *img_frame);

	/**
	 * Initialize particle cloud.
	 * @param tracked_object_histogram		histogram of the entity that needs to be tracked
	 * @param coord							CImg coordinates, careful: picks 0,1 3,4 (skips 2)
	 * @param particle_count				the number of particles to be generated
	 * @return void
	 */
	void Init(NormalizedHistogramValues &tracked_object_histogram, CImg<CoordValue> &coord,
			int particle_count);

	//! Transition of all particles following a certain motion model
	void Transition();

	/**
	 * Autoregressive model to estimate where an object will be next. See implementation
	 * for the actual model used.
	 */
	ParticleState *Transition(ParticleState oldp);

	/**
	 * Update particle itself
	 */
	void doTransition(ParticleState &oldp);

	/**
	 * The likelihood of a player at all locations in the image using a given region size.
	 */
	void Likelihood(RegionSize region_size);

	/**
	 * Calculate likelihood of all particles
	 */
	void Likelihood();

	/**
	 * Return particles, or more specific, return the coordinates of the particles, ordered
	 * on weight.
	 */
	void GetParticleCoordinates(std::vector<CImg<CoordValue> *> & coordinates);
protected:

	/**
	 * Calculate the likelihood of a player and the state indicated by the parameter
	 * "state" which contains an x and y position, a width and a height. This is used
	 * to define a rectangle for which a histogram is matched against the reference
	 * histogram of the object that is tracked.
	 * @param state			the state of the particle (position, width, height)
	 * @return				conceptual "distance" to the reference (tracked) object
	 */
	float Likelihood(ParticleState & state);

private:
	//! The number of bins
	int bins;

	//! The histogram of the object to be tracked
	NormalizedHistogramValues tracked_object_histogram;

	//! Image data
	pDataMatrix data;

	//! Image to get data from
	CImg<DataValue> * img;

	//! Seed for random number generator
	int seed;

	//! See http://demonstrations.wolfram.com/AutoRegressiveSimulationSecondOrder/
	std::vector<Value> auto_coeff;


};

#endif /* POSITIONPARTICLEFILTER_HPP_ */
