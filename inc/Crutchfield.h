/**
 * @brief 
 * @file Crutchfield.h
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
 * @date    Aug 6, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
 */


#ifndef CRUTCHFIELD_H_
#define CRUTCHFIELD_H_

// General files
#include <Histogram.h>
#include <DistanceSource.h>

#ifndef CAREFUL_USAGE
#define CHECK_DIST
#else
#define CHECK_DIST { assert (dist != NULL); }
#endif

/* **************************************************************************************
 * Interface of Crutchfield
 * **************************************************************************************/

/**
 * The Crutchfield distance metric is not so sophisticated anymore, once the probabilities
 * and conditional probabilities between sensors is available.
 *
 * Adds "distance" information considering the joint/conditional probabilities between
 * sensor sources.
 *
 * Provides:
 * - calcDistances
 * - getDistance for each sensor pair
 *
 */
class Crutchfield: public Histogram, public DistanceSource {
public:
	//! Constructor Crutchfield
	Crutchfield(int bins, int width, int height);

	//! Destructor ~Crutchfield
	virtual ~Crutchfield();

	/**
	 * Deallocate the internally used distance matrix.
	 */
	void Clear();

	/**
	 * Calculate all distances. It is necessary to call first
	 *     "calcProbabilities(DataFrames & frames)"
	 * from the parent Histogram class.
	 *
	 * This sets all the (conditional) probabilities that are required to do the distance
	 * calculations. The result is a side-effect in the form of filling a data structure.
	 */
	void calcDistances();

	/**
	 * Calculate the Crutchfield distance between two sensors
	 */
	Value calcDistance(int p0, int p1);

	/**
	 * Get previously calculated distance (with calcDistance)
	 */
	Value getDistance(int p0, int p1) {
		CHECK_DIST;
		return dist[p0*p_size+p1];
	}

protected:

	//! Distances matrix (these are actual floats)
	Value *dist;

private:


};

#endif /* CRUTCHFIELD_H_ */
