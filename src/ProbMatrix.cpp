/**
 * @brief 
 * @file ProbMatrix.cpp
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

#include <ProbMatrix.h>
#include <stddef.h>

/* **************************************************************************************
 * Implementation of ProbMatrix
 * **************************************************************************************/

ProbMatrix::ProbMatrix(int bins, int width, int height): bins(bins),
	p_height(height),
	p_width(width),
	p_size(height * width),
	bins_squared(bins*bins),
	freq(NULL),
	joint_freq(NULL),
	frame_count(0) {

}

ProbMatrix::~ProbMatrix() {

}
