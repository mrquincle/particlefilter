/**
 * @brief 
 * @file Crutchfield.cpp
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

#include <Crutchfield.h>

#include <iostream>
#include <assert.h>

using namespace std;

/* **************************************************************************************
 * Implementation of Crutchfield
 * **************************************************************************************/

Crutchfield::Crutchfield(int bins, int width, int height): Histogram(bins, width, height),
		DistanceSource(),
		dist(NULL) {

}

Crutchfield::~Crutchfield() {
	Clear();
}

void Crutchfield::Clear() {
	if (dist != NULL) {
		delete [] dist;
	}
	dist = NULL;
}

/**
 * Calculates all mutual distances between "sensors". This will fill the "dist" matrix. We
 * will loop over all possible sensor pairs and calculate their distance (considering a
 * series of images) using the calcDistance function.
 *
 * The only thing you might to take a look at is the default binning procedure.
 */
void Crutchfield::calcDistances() {
	CHECK_DIST;
	Clear();

	cout << "Calculate distances" << endl;
	dist = new Value[p_size * p_size];
	for (int p0 = 0; p0 < p_size; p0++) {
		for (int p1 = 0; p1 < p_size; p1++) {
			dist[p0*p_size+p1] = calcDistance(p0, p1);
		}
	}
}


/**
 * Get distance between pixel p0 and pixel p1 given a sequence of images. Every pixel is
 * considered a separate sensor. The distance between pixels can be used to decide if a
 * camera is mounted on the same side of the organism using only visual input.
 *
 *  d(X,Y) = H(X|Y) + H(Y|X)
 *
 * Where d is the Crutchfield distance and H is conditional entropy of one information source
 * given the other.
 */
Value Crutchfield::calcDistance(int p0, int p1) {
	Value dist = getConditionalEntropy(p0, p1) +
			getConditionalEntropy(p1, p0);
	assert (dist >= 0);
	return dist;
}
