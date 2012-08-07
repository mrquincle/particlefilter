/**
 * @file Crutchfield.cpp
 * @brief Calculate Crutchfield distance metric to sensors
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
 * @date	Feb 22, 2011
 * @project	Replicator FP7
 * @company	Almende B.V.
 * @case	Sensory processing
 */

#include <Histogram.h>

// General files
#include <iostream>
#include <assert.h>
#include <math.h>

using namespace std;

/* **************************************************************************************
 * Implementation of Histogram
 * **************************************************************************************/

/**
 * For histogram calculations we only need to count
 */
Histogram::Histogram(int bins, int width, int height): ProbMatrix(bins, width, height) {

}

/**
 * The destructor deallocates everything.
 */
Histogram::~Histogram() {
	Clear();
}

/**
 * Delete all arrays that might have been created to calculate the histograms.
 */
void Histogram::Clear() {
	if (freq != NULL) {
		delete [] freq;
	}
	if (joint_freq != NULL) {
		delete [] joint_freq;
	}
}

/**
 * We need a function that calculates the conditional probability for two sensors over
 * a time series. We calculate this for all sensor values and calculate the conditional
 * entropy. A structure is needed to store all sensor values over a certain time.
 *
 * @return void
 *
 * Resulting side-information:
 * - frequency matrix (histogram)
 * - joint frequency matrix (conditional histogram)
 * - number of frames (frame_count)
 */
void Histogram::calcProbabilities(DataFrames & frames) {
	frame_count = frames.size();

	// Delete previous matrices
	Clear();

	// Create probability matrix
	cout << "Create probability matrix of size " << bins << "x" << p_size << endl;
	freq = new HistogramValue[bins * p_size];
	if (freq == NULL) {
		cerr << "Probability matrix could not be allocated" << endl;
	}

	// Fill probability matrix
	cout << "Fill probability matrix" << endl;
	for (int t = 0; t < frame_count; ++t) {
		for (int p = 0; p < p_size; ++p) {
			pDataMatrix data = frames[t];
			Value v = data[p];
			int bin = (v * bins) / 256;
			assert (bin < 256);
			freq[p*bins+bin]++;
		}
	}

	// Create joint probability matrix
	cout << "Create joint probability matrix of size " << bins << "x" << bins << " * " << p_size * p_size << endl;
	joint_freq = new HistogramValue[bins_squared * p_size * p_size];
	if (joint_freq == NULL) {
		cerr << "Joint probability matrix could not be allocated" << endl;
	}

	// Fill joint probability matrix
	cout << "Fill joint probability matrix" << endl;
	for (int t = 0; t < frame_count; ++t) {
		for (int p0 = 0; p0 < p_size; ++p0) {
			for (int p1 = 1; p1 < p_size; ++p1) {
				if (p0 <= p1) continue; // omit filling in bottom triangle of matrix
				pDataMatrix data = frames[t];
				Value v0 = data[p0];
				Value v1 = data[p1];
				int bin0 = (v0 * bins) / 256;
				int bin1 = (v1 * bins) / 256;
				int m = p0*bins_squared + p1*bins_squared*p_size; //offset
				joint_freq[m+bin0+bins*bin1]++;
#ifdef VERBOSE
				cout << "Calculate bin " << bin1 << "=" << s1.val[0] << "*" << bins << "/256" << endl;
				cout << "Joint freq " << p0 << "," << p1 << ":" << bin0 << "," << bin1 << " ++" << endl;
				cout << "Thus m becomes " << p0*bins*bins << "+" << p1*bins*bins*p_width*p_height << "=" << m << endl;
				cout << "Increment position m+" << bin0*bins << "+" << bin1 << "=" << m+bin0*bins+bin1 << endl;
#endif
			}
		}
	}
}


/**
 * Calculates conditional entropy of sensors with respect to each other for a time
 * series of length "frame_count".
 *
 *   H(Y|X) = - SUM_x SUM_y p(x,y) log { p(x) / p(x,y) }
 *
 * where H is the entropy, SUM is a sigma sign, summing over all possibilities x
 * and y, log is log2, p(x,y) is the joint probability of P(X=x,Y=y). Here X is a given
 * information source, a sensor, and x is the number of states this information source
 * can occupy, in this case sensor values are divided into N bins.
 *
 * @param p0	Index of first sensor
 * @param p1	Index of second sensor
 * @result		Conditional entropy
 *
 * Required side-information:
 * - previously calculated frequency matrix
 * - previously calculated joint frequency matrix
 * - previously set number of frames "frame_count"
 */
Value Histogram::getConditionalEntropy(int p0, int p1) {
	CHECK_FRAMECOUNT;
	CHECK_FREQ;
	CHECK_JOINTFREQ;
	Value sum = 0;
	if (p0 == p1) return 0;
	for (int b0 = 0; b0 < bins; b0++) {
		for (int b1 = 0; b1 < bins; b1++) {
			Value f01 = (Value)getJointFrequency(p0, b0, p1, b1);
			Value f0 = (Value)getFrequency(p0, b0);

			// remove division by 0 and log(0)
			if ( (f0 == 0) || (f01 == 0) ) continue;
			// use log = ln for now
			sum += (f01 / (Value)frame_count) * log ( f0 / f01 ) / log(2);
		}
	}
	return sum;
}

#ifdef DEBUG

void Histogram::printFrequencies(int bin) {
	for (int i = 0; i < p_width; i++) {
		for (int j = 0; j < p_height; j++) {
			int p = j * p_width + i;
			cout << freq[p*bins+bin] << ", ";
		}
	}
	cout << endl;
}

void Histogram::printJointFrequencies(int bin0, int bin1) {
	for (int i0 = 0; i0 < p_width; i0++) {
		for (int j0 = 0; j0 < p_height; j0++) {
			for (int i1 = 0; i1 < p_width; i1++) {
				for (int j1 = 0; j1 < p_height; j1++) {
					int p0 = j0 * p_width + i0;
					int p1 = j1 * p_width + i1;
					int m = p0*bins*bins+p1*bins*bins*p_width*p_height; //offset
					cout << joint_freq[m+bin0+bins*bin1] << ", ";
				}
			}
			cout << endl;
		}
	}
}

void Histogram::printJointFrequenciesForPixels(int p0, int p1) {
	for (int bin0 = 0; bin0 < bins; bin0++) {
		for (int bin1 = 0; bin1 < bins; bin1++) {
			int m = p0*bins*bins+p1*bins*bins*p_width*p_height;
			cout << joint_freq[m+bin0+bins*bin1] << ", ";
		}
		cout << endl;
	}
}

void Histogram::printJointFrequencies(char printmode) {
	switch(printmode) {
	case 0: {
		int w = bins*bins;//*p_width*p_height;
		int h = p_width*p_height*p_width*p_height;
		cout << "Width = " << w << " and height = " << h << endl;
		for (int i = 0; i < h; i++) {
			for (int j = 0; j < w; j++) {
				cout << joint_freq[i*w+j] << ", ";
			}
			cout << endl;
		}
		break;
	}
	case 1: {
		for (int bin0 = 0; bin0 < bins; bin0++) {
			for (int bin1 = 0; bin1 < bins; bin1++) {
				cout << "bin[" << bin0 << ", " << bin1 << "]: " << endl;
				printJointFrequencies(bin0, bin1);
			}
		}
		break;
	}
	case 2: {
		int w = p_width*p_height;
		int h = p_width*p_height;
		cout << "Width = " << w << " and height = " << h << " and #bins = " << bins << endl;
		for (int i = 0; i < h; i++) {
			for (int j = 0; j < w; j++) {
				cout << "pixel pair {" << i << ", " << j << "}" << endl;
				printJointFrequenciesForPixels(i, j);
			}
		}
		break;
	}
	default: cerr << "Huh?" << endl;
	break;
	}
}

/**
 * Print distances
 */
void Histogram::printDistances() {
	cout << "All distances:" << endl;
	for (int i0 = 0; i0 < p_width; i0++) {
		for (int j0 = 0; j0 < p_height; j0++) {
			for (int i1 = 0; i1 < p_width; i1++) {
				for (int j1 = 0; j1 < p_height; j1++) {
					int p0 = j0 * p_width + i0;
					int p1 = j1 * p_width + i1;
					cout << (int)dist[p0*p_width*p_height+p1] << ", ";
				}
			}
			cout << endl;
		}
	}
}

pDataMatrix Histogram::drawDistances() {
	pDataMatrix result = new Value[p_size*p_size];
	for (int p0 = 0; p0 < p_size; ++p0) {
		for (int p1 = 0; p1 < p_size; ++p1) {
			Value d = getDistance(p0, p1);
			result[p0 * p_size + p1] = d;
		}
	}
	return result;
}


void Histogram::printBins(pDataMatrix data) {
	if (p_width == 0) return;
	for (int i = 0; i < p_width; i++) {
		for (int j = 0; j < p_height; j++) {
			int p = j * p_width + i;
			Value v = data[p];
			int bin = (v * bins) / 256;
			cout << bin << " ";
		}
		cout << endl;
	}
}

#endif
