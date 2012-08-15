/**
 * @brief 
 * @file ProbMatrix.h
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


#ifndef PROBMATRIX_H_
#define PROBMATRIX_H_

/* **************************************************************************************
 * Configuration options
 * **************************************************************************************/

#include <Config.h>

//! Remove joint probability calculations, which saves a lot on memory requirements
#define CALC_JOINTFREQ 0

/* **************************************************************************************
 * Configuration option consequences
 * **************************************************************************************/

#if CALC_JOINTFREQ == 0
#undef CALC_JOINTFREQ
#endif

#ifndef CAREFUL_USAGE
#define CHECK_FREQ
#define CHECK_JOINTFREQ
#define CHECK_FRAMECOUNT
#else
#define CHECK_FREQ { assert (freq != NULL); }
#define CHECK_JOINTFREQ { assert (joint_freq != NULL); }
#define CHECK_FRAMECOUNT { assert (frame_count > 0); }
#endif


/* **************************************************************************************
 * Includes
 * **************************************************************************************/

// General files
#include <vector>

/* **************************************************************************************
 * General data types/formats
 * **************************************************************************************/

/**
 * Probabilities need to represented by floats or doubles.
 */
typedef float Value;

/**
 * It is possible to represent the incoming data by floats, doubles, chars, integers,
 * whatever floats your boat. :-)
 */
typedef unsigned char DataValue;

/**
 * For just counting (to create histogram) you will not need floating point values. We will
 * use integers for that.
 */
typedef int HistogramValue;

/**
 * The data is just represented as a big char/float/double vector. To remember the fact
 * that it is already a pointer to a series of values, it gets the prefix "p". It can just
 * be passed like function(pDataMatrix data) and put in a vector as vector<pDataMatrix>, because it
 * doesn't actually carry the values "with it".
 */
typedef DataValue* pDataMatrix;

/**
 * A (successive) series of data frames is a vector, with each element referring to an array
 * of doubles/floats/integers as defined by "Value".
 */
typedef std::vector<pDataMatrix> DataFrames;


/* **************************************************************************************
 * Interface of ProbMatrix
 * **************************************************************************************/

/**
 * This sets up the data structures for calculations with respect to frequencies and probabilities.
 * It does not perform these calculations itself.
 */
class ProbMatrix {
public:
	//! Constructor ProbMatrix
	ProbMatrix(int bins, int width, int height);

	//! Destructor ~ProbMatrix
	virtual ~ProbMatrix();

	//! Get frequency of certain pixel in certain state/bin
	inline HistogramValue getFrequency(int p, int bin) {
		CHECK_FREQ;
		return freq[p*bins+bin];
	}

	//! Get probability for pixel in state bin
	//! Try to use getFrequency instead and only divide by t_len later on
	inline Value getProbability(int p, int bin) {
		CHECK_FREQ;
		CHECK_FRAMECOUNT;
		return freq[p*bins+bin] / (Value)frame_count;
	}

	//! Get number of sensors
	inline int getSensorCount() { return p_size; }

	//! Get number of bins
	inline int getBins() { return bins; }

	//! Set number of bins
	inline void setBins(int bins) { this->bins = bins; bins_squared = bins*bins; }

	//! Get joint probability for pixel p0 and p1 in state bin0 and bin1
	//! Try to use getJointFrequency instead and divide by time_len as
	//! late as possible
	Value getJointProbability(int p0, int bin0, int p1, int bin1) {
		CHECK_FRAMECOUNT;
		return getJointFrequency(p0, bin0, p1, bin1) / (Value)frame_count;
	}

	/**
	 * Get joint "probability". As you can see it is called joint frequency and not joint probability.
	 * This means that we do not divide by the total number of possible events (in other words, divide
	 * by the number of frames. This makes it possible to use integers, and we do not need floats/doubles.
	 * If you really want to use probabilities, use getJointProbability instead (which of course will use
	 * the previously set number of frames from the calculations before).
	 *
	 * The getJointFrequency function just returns values that are already pre-calculated. Each time
	 * the input changes, you will have to call calcProbabilities first.
	 */
	HistogramValue getJointFrequency(int p0, int bin0, int p1, int bin1) {
		CHECK_JOINTFREQ;
		if (p0 == p1) return 0;
		if (p0 < p1) { // swap, only upper triangle of matrix is filled, because it is symmetric
			int temp = p0; p0 = p1; p1 = temp;
			temp = bin0; bin0 = bin1; bin1 = temp;
		}
		// get offset for joint freq vector for (p0,p1)
		// p_size is needed for row size estimation
		int m = p0*bins_squared + p1*bins_squared*p_size;
		return joint_freq[m+bin0+bins*bin1];
	}

protected:
	//! Number of bins
	int bins;

	//! Width of individual data frame
	int p_width;

	//! Height of individual data frame
	int p_height;

	//! Number of bins squared (automatically calculated)
	int bins_squared;

	//! The total size of one data frame (automatically calculated)
	int p_size;

	//! Probability (or actually frequency) matrix
	HistogramValue *freq;

	//! Joint probability (or actually frequency) matrix
	HistogramValue *joint_freq;

	//! Number of frames
	int frame_count;

private:

};

#endif /* PROBMATRIX_H_ */
