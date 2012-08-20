/**
 * @brief 
 * @file testHistogram.cpp
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

#include <Histogram.h>
#include <Container.hpp>

#include <iostream>

using namespace std;

void test_histogram() {
	cout << " === start test histogram === " << endl;

	for (int i = 0; i < 2; ++i) {
		int size = 10;
		int bins = 4;
		Histogram histogram(bins, size, 1);

		DataFrames frames;
		int nof_frames = 1;
		for (int f = 0; f < nof_frames; ++f) {
			pDataMatrix data = new DataValue[size];
			for (int i = 0; i < size; ++i) {
				data[i] = 40 * ((i % 2) + 1); // set values 40 or 80
			}
			frames.push_back(data);
		}

		histogram.calcProbabilities(frames);

		std::vector<Value> result;
		histogram.getProbabilities(result);

		cout << "Result: ";
		for (int i = 0; i < result.size(); ++i) {
			cout << result[i] << ' ';
		}
		cout << endl;
	}
	cout << " === end test histogram === " << endl;
}
