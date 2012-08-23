/**
 * @brief 
 * @file testConvolution.h
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
 * @date    Aug 23, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
 */


#ifndef TESTCONVOLUTION_H_
#define TESTCONVOLUTION_H_

#include <Container.hpp>
#include <Print.hpp>

using namespace dobots;
using namespace std;

void test_convolution() {
	std::vector<int> vec1;
	std::vector<int> vec2;
	std::vector<int> vec3;
	vec1.clear();
	vec2.clear();
	vec3.clear();
	int size = 4;

	for (int i = 0; i < size; ++i)
		vec1.push_back(i+1);

	cout << "Vector 1: ";
	print(vec1.begin(), vec1.end());

	for (int i = 0; i < size; ++i)
		vec2.push_back(i+1);

	cout << "Vector 2: ";
	print(vec2.begin(), vec2.end());

	vec3.resize(size);

	integral(vec1.begin(), vec1.end(), vec2.begin(), vec3.begin());
	cout << "Integral: ";
	print(vec3.begin(), vec3.end());

	clean(vec3.begin(), vec3.end());
	cauchy_product(vec1.begin(), vec1.end(), vec2.end(), vec3.begin());
	cout << "Cauchy: ";
	print(vec3.begin(), vec3.end());

	// circular convolution, check quickly with octave
	//      vec3 = ifft(fft(vec1).*fft(vec2)): 26 28 26 20
	clean(vec3.begin(), vec3.end());
	circular_convolution(vec1.begin(), vec1.end(), vec2.begin(), vec2.end(), vec3.begin(), 1);
	cout << "Circular convolution: ";
	print(vec3.begin(), vec3.end());

	cout << "Vector 2: ";
	print(vec2.begin(), vec2.end());
}

#endif /* TESTCONVOLUTION_H_ */
