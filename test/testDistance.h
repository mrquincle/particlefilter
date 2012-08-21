/**
 * @brief 
 * @file testDistance.h
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
 * @date    Aug 21, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
 */


#ifndef TESTDISTANCE_H_
#define TESTDISTANCE_H_

#include <Container.hpp>

#include <cassert>

using namespace std;
using namespace dobots;

#define TESTVALUE	double
#define TESTPOINT	vector
#define TESTSET 	set

#define TESTPOINT_DEF		std::TESTPOINT < TESTVALUE >
#define TESTSET_DEF  		std::TESTSET<TESTPOINT_DEF * >

#define TESTPOINT_ITER	TESTPOINT_DEF::iterator
#define TESTSET_ITER		TESTSET_DEF::iterator

void test_distance() {
	cout << " === start test distance === " << endl;

	TESTSET_DEF set0;
	TESTPOINT_DEF *p = new TESTPOINT_DEF();

	p->push_back(1);

	TESTPOINT_DEF *p0 = new TESTPOINT_DEF();
	TESTPOINT_DEF *p1 = new TESTPOINT_DEF();
	p0->clear();
	p1->clear();
	p0->push_back(3);
	p1->push_back(6);
	set0.clear();
	set0.insert(p0);
	set0.insert(p1);

	SetDistanceMetric set_metric = SDM_INFIMIM;
	DistanceMetric point_metric = DM_EUCLIDEAN;
	TESTVALUE result;
	result = dobots::distance_to_point<TESTVALUE>(set0.begin(), set0.end(), p->begin(), p->end(), set_metric, point_metric);

	cout << "Inf result d(1,[3,6]) = " << result << " and should be 2 (minimum distance is between 1 and 3)" <<  endl;
	assert (result == 2);

	TESTPOINT_DEF *p2 = new TESTPOINT_DEF();
	TESTPOINT_DEF *p3 = new TESTPOINT_DEF();
	TESTPOINT_DEF *p4 = new TESTPOINT_DEF();
	TESTPOINT_DEF *p5 = new TESTPOINT_DEF();
	TESTSET_DEF set1;
	set0.clear();
	set1.clear();
	p0->clear();
	p1->clear();
	p2->clear();
	p3->clear();
	p4->clear();
	p5->clear();
	p0->push_back(1);
	p1->push_back(3);
	p2->push_back(6);
	p3->push_back(7);
	set0.insert(p0); set0.insert(p1); set0.insert(p2); set0.insert(p3);
	p4->push_back(3);
	p5->push_back(6);
	set1.insert(p4);
	set1.insert(p5);

	set_metric = SDM_SUPINF;
	result = dobots::distance_to_set<TESTVALUE, TESTSET_ITER, TESTPOINT_ITER>(set0.begin(), set0.end(), set1.begin(), set1.end(), set_metric, point_metric);

	cout << "SupInf result d([1,3,6,7], [3,6]) = " << result << " and should be 2 :::: ";
	cout << "d(1,[3,6])=2, d(3,[3,6])=0, d(6,[3,6])=0, d(7,[3,6])=1 (maximum value here is 2)" << endl;
	assert (result == 2);

	result = dobots::distance_to_set<TESTVALUE, TESTSET_ITER, TESTPOINT_ITER>(set1.begin(), set1.end(), set0.begin(), set0.end(), set_metric, point_metric);

	cout << "SupInf result d([3,6], [1,3,6,7]) = " << result << " and should be 0 :::: ";
	cout << "d(3,[1,3,6,7])=0, d(6,[1,3,6,7])=0 (maximum value here is 0)" << endl;
	assert (result == 0);

	set_metric = SDM_HAUSDORFF;
	result = dobots::distance_to_set<TESTVALUE, TESTSET_ITER, TESTPOINT_ITER>(set1.begin(), set1.end(), set0.begin(), set0.end(), set_metric, point_metric);

	cout << "Hausdorff result d([3,6], [1,3,6,7]) = " << result << " and should be 2 again (maximum of above)" << endl;
	assert (result == 2);


	delete p;
	delete p0;
	delete p1;
	delete p2;
	delete p3;
	delete p4;
	delete p5;
	set0.clear();
	set1.clear();

	cout << " === end test distance metrics === " << endl;
}


#endif /* TESTDISTANCE_H_ */
