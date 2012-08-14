/**
 * @brief 
 * @file testRegression.cpp
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

#include <Autoregression.hpp>
#include <Print.hpp>
#include <iostream>

using namespace std;

void test_autoregression() {
	cout << " === start test histogram === " << endl;

	cout << "[*] test predict" << endl;
	std::vector<double> x;
	x.clear();
	std::vector<double> c;
	c.clear();
	c.push_back(1);
	c.push_back(-2);
	cout << "Coefficients: ";
	dobots::print(c.begin(), c.end());

	double result = 0.0;
	x.push_back(1);
	x.push_back(1.2);
	x.push_back(1.4);

	double sum = std::inner_product(x.begin(), x.end(), c.begin(), 0.0);
	cout << "Inner product: " << sum << endl;

	for (int i = 0; i < 10; ++i) {
		result = dobots::predict(x.begin(), x.end(), c.begin(), 0.0);
		dobots::pushpop(x.begin(), x.end(), result);
	}
	cout << "Predicted values after 10 time steps: ";
	dobots::print(x.begin(), x.end());

	cout << "[*] test print and rotate and pushpop" << endl;
	x.clear();
	for (int i = 0; i < 10; ++i) {
		x.push_back(i);
	}
	dobots::print(x.begin(), x.end());
	dobots::pushpop(x.begin(), x.end(), 30);
	dobots::pushpop(x.begin(), x.end(), 40);
	dobots::print(x.begin(), x.end());

	cout << " === end test histogram === " << endl;
}
