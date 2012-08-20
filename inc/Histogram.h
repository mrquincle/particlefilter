/**
 * @file Crutchfield.h
 * @brief 
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
 * Copyright © 2012 Anne van Rossum <anne@almende.com>
 *
 * @author 	Anne C. van Rossum
 * @date	Aug 6, 2012
 * @project	Replicator FP7
 * @company	Almende B.V.
 * @case	
 */
#ifndef HISTOGRAM_H_
#define HISTOGRAM_H_

#include <ProbMatrix.h>
#include <vector>
#include <assert.h>

//! Container for histogram values
typedef std::vector<HistogramValue> HistogramValues;

//! Container for normalised histogram values
typedef std::vector<Value> NormalizedHistogramValues;

/* **************************************************************************************
 * Interface of Histogram
 * **************************************************************************************/

/**
 * This class calculates Histogram from data provided in the form of "matrices". It calculates
 * frequencies, and not probabilities directly. However, the latter can be easily obtained by
 * properly normalising the frequencies. For that the number of frames is used.
 *
 * Usage:
 *   calcProbabilities
 * Do not call the getX functions before the calcX functions.
 */
class Histogram: public ProbMatrix {
public:
	//! Constructor with number of bins and the size of the frames
	Histogram(int bins, int width, int height);

	//! Destructor
	virtual ~Histogram();

	/**
	 * Deallocate structures, used in destructor and on calcProbabilities.
	 */
	void Clear();

	/**
	 *  Calculate probabilities of value given a series of frames with (different) values at that
	 *  same position in the frame. This is enough to define a histogram over a data frame. This
	 *  will calculate the probabilities of occurrence of individual values in a certain bin, but
	 *  also the joint probabilities with respect to all other values. So, this will generate
	 *  internally two data structures, one of size of the data (D), and one quadratic with
	 *  respect to that (DxD).
	 *
	 *  The results are in the form of "side effects" to this function. Internal data structures
	 *  are filled. To get the calculated entities, use for example:
	 *  - getProbability
	 *  - getJointProbability
	 *  - getConditionalEntropy
	 */
	void calcProbabilities(DataFrames & frames);

	/**
	 * The conditional entropy can be obtained after calcProbability. It does not actually
	 * calculate the entropy itself, you will need to call calcProbabilities before!
	 */
	Value getConditionalEntropy(int p0, int p1);

	/**
	 * Calculation that calculates the bin index given a specific value. Currently very simple
	 * just uniformly dividing bins over the entire expected range from [0,255] of values.
	 */
	inline int value2bin(Value v) {
		assert (v >= 0);
		int bin = (v * bins) / 256;
		assert (bin < bins);
		assert (bin < 256);
		return bin;
	}

	/**
	 * The other (default) function returns the frequency of data events of an individual sensor over
	 * time. A value is counted several times to fall into bin X over a set of frames, and then
	 * counted a few times to fall into bin Y over another set of frames.
	 *
	 * Contrary to that, this function does forget about individual sensors and returns probability
	 * measures over all sensors (and over time).
	 */
	void getFrequencies(HistogramValues &bin_result);

	//! Get total number of samples that are recorded
	int getSamples();

	/**
	 * Exactly the same as getFrequencies, but now normalised with the sum of all events.
	 */
	void getProbabilities(NormalizedHistogramValues &bin_result);
#ifdef DEBUG
	//! Print distances
	void printDistances();

	//! Print bin values
	void printBins(pDataMatrix data);

	void printFrequencies(int bin);

	void printJointFrequencies(char printmode = 1);

	void printJointFrequencies(int bin0, int bin1);

	pDataMatrix drawDistances();
protected:
	void printJointFrequenciesForPixels(int p0, int p1);

#endif
};

#endif /* HISTOGRAM_H_ */
