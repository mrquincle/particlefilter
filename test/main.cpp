/**
 * @brief 
 * @file main.cpp
 *
 * This file is created at Almende B.V. It is open-source software and part of the Common 
 * Hybrid Agent Platform (CHAP). A toolbox with a lot of open-source tools, ranging from 
 * thread pools and TCP/IP components to control architectures and learning algorithms. 
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless,
 * we personally strongly object against this software being used by the military, in the
 * bio-industry, for animal experimentation, or anything that violates the Universal
 * Declaration of Human Rights.
 *
 * Copyright © 2012 Anne van Rossum <anne@almende.com>
 *
 * @author  Anne C. van Rossum
 * @date    Jul 25, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
 */

#include <ParticleFilter.hpp>

/* **************************************************************************************
 * Implementation of main
 * **************************************************************************************/

struct Position {
	int x;
	int y;
	int width;
	int height;
	int histogram;
};

class PositionParticleFilter: public ParticleFilter<Position> {
public:
	void Transition() {

	}
};

/**
 * Showcase for a particle filter. It - for that reason - uses images, because it is
 * easiest to tell what is going on, using image processing.
 */
int main() {
	PositionParticleFilter pf;

//	pf.
}

