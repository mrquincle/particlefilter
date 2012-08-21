/**
 * @brief Particle Filter for Position Estimates on Images
 * @file PositionParticleFilter.h
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


#ifndef POSITIONPARTICLEFILTER_H_
#define POSITIONPARTICLEFILTER_H_

#include <ParticleFilter.hpp>
#include <CImg.h>

#include <Histogram.h>
#include <Container.hpp>
#include <Autoregression.hpp>

#include <algorithm>
#include <cassert>

using namespace cimg_library;
using namespace std;

//! An "unsigned char" is not be enough for resolution larger than 256x256, so we go for an "int"
typedef int CoordValue;

/**
 * Just the extend of a region, not its location.
 */
struct RegionSize {
	int width;
	int height;
};

static int ParticleStateId = 0;

/**
 * The particle's state is just a rectangular region, and is defined over a few time steps.
 */
class ParticleState {
public:
	ParticleState() {
		id = ++ParticleStateId;
		x.clear();
		y.clear();
		scale.clear();
		likelihood = 0;
		width = 0;
		height = 0;
	}

	ParticleState(int id): id(id) {
		x.clear();
		y.clear();
		scale.clear();
		likelihood = 0;
		width = 0;
		height = 0;
	}

	~ParticleState() {
		x.clear();
		y.clear();
		scale.clear();
	}

	//! The (default) width of the rectangular region
	int width;
	//! The (default) height of the rectangular region
	int height;

	//! The histogram corresponding to a particle
	//	int histogram;
	Value likelihood;

	//! The x-vector denotes the horizontal centre of a rectangular region and its history
	std::vector<Value> x;
	//! The y-vector denotes the vertical centre of a rectangular region and its history
	std::vector<Value> y;
	//! A floating point value that scales width and height
	std::vector<Value> scale;

	//! Easy printing
	friend std::ostream& operator<<(std::ostream& os, const ParticleState & ps) {
		if ((ps.x.size() == 1) && (ps.y.size() == 1)) {
			os << ps.id << " [" << ps.x.front() << ',' << ps.y.front() << "] (" << ps.likelihood << ")";
		} else if (ps.x.size() > 1 && ps.y.size() > 1) {
			// use history at position [1]
			os << ps.id << " [" << ps.x[1] << ',' << ps.y[1] << "] -> [" << ps.x[0] << ',' << ps.y[0] << "] (" << ps.likelihood << ")";
		} else {
			os << ps.id << " []";
		}
		return os;
	}

	/**
	 * A copy instructor for ParticleState. Somehow we have to build up the vectors from
	 * scratch and cannot use std::copy.
	 */
	ParticleState(const ParticleState & other): x(other.x.size()),
			y(other.y.size()), scale(other.scale.size()) {
		id = other.id;
		width = other.width;
		height = other.height;
		likelihood = other.likelihood;

		x.clear(); y.clear(); scale.clear();
		for (int i = 0; i < other.x.size(); ++i) x.push_back(other.x[i]);
		for (int i = 0; i < other.y.size(); ++i) y.push_back(other.y[i]);
		for (int i = 0; i < other.scale.size(); ++i) scale.push_back(other.scale[i]);

		assert (!other.x.empty());
		ASSERT_EQUAL(x.size(), other.x.size());
	}

	inline const int getId() { return id; }

private:
	//! Mainly for debugging reasons, make sure we actually make copies at the right moment, etc.
	int id;
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
	void Tick(CImg<DataValue> *img_frame, int subticks = 1);

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
	void Transition(ParticleState &oldp);

	/**
	 * Calculate likelihood of all particles
	 */
	void Likelihood();

	/**
	 * Return particles, or more specific, return the coordinates of the particles, ordered
	 * on weight.
	 */
	void GetParticleCoordinates(std::vector<CImg<CoordValue> *> & coordinates);

	/**
	 * Return the likelihood of the histogram at all possible positions.
	 */
	void GetLikelihoods(CImg<DataValue> & result, RegionSize region_size, int block_size = 8);
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
//	pDataMatrix data;

	//! Image to get data from
	CImg<DataValue> * img;

	//! Seed for random number generator
	int seed;

	//! See http://demonstrations.wolfram.com/AutoRegressiveSimulationSecondOrder/
	std::vector<Value> auto_coeff;


};

#endif /* POSITIONPARTICLEFILTER_H_ */
