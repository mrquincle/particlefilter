/**
 * @file DistanceSource.h
 * @brief  Abstract class for algorithms that generate distances between sensors
 *
 * This file is created at Almende B.V. It is open-source software and part of the Common 
 * Hybrid Agent Platform (CHAP). A toolbox with a lot of open-source tools, ranging from 
 * thread pools and TCP/IP components to control architectures and learning algorithms. 
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless,
 * we personally strongly object against this software used by the military, in the
 * bio-industry, for animal experimentation, or anything that violates the Universal
 * Declaration of Human Rights.
 *
 * Copyright © 2010 Anne van Rossum <anne@almende.com>
 *
 * @author 	Anne C. van Rossum
 * @date	Feb 23, 2011
 * @project	Replicator FP7
 * @company	Almende B.V.
 * @case	
 */


#ifndef DISTANCESOURCE_H_
#define DISTANCESOURCE_H_

// General files

/* **************************************************************************************
 * Interface of DistanceSource
 * **************************************************************************************/

/**
 * Abstract class that defines getDistance() for two given sensors.
 */
class DistanceSource {
public:
	//! Constructor DistanceSource
	DistanceSource() {}

	//! Destructor ~DistanceSource
	virtual ~DistanceSource() {}

	//! Get distance
	virtual float getDistance(int sensor0, int sensor1) = 0;

	//! Get number of sensors
	virtual int getSensorCount() = 0;
protected:

private:

};

#endif /* DISTANCESOURCE_H_ */
