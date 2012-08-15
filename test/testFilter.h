/**
 * @brief 
 * @file testFilter.h
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
 * @date    Aug 15, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
 */

#include <ParticleFilter.hpp>
#include <Print.hpp>
#include <cassert>

using namespace dobots;

/**
 * If you use a stream operator to print the particles, you will need to define it for
 * your state variable too.
 */
struct TestData {
	int fieldA;
	int fieldB;

	friend std::ostream& operator<<(std::ostream& os, const TestData & p) {
		os << p.fieldA << ',' << p.fieldB;
		return os;
	}
};

class TestParticleFilter: public ParticleFilter<TestData> {
public:
	TestParticleFilter() {
		particle_count = 10;
	}

	~TestParticleFilter() {}

	void Init() {
		for (int i = 1; i < particle_count+1; ++i) {
			Particle<TestData> *p = new Particle<TestData>();
			TestData *data = new TestData();
			data->fieldA = i; data->fieldB = i;
			p->setState(data);
			p->setWeight(i); // unnormalized
			getParticles().push_back(p);
		}
	}

	void Print() {
		std::cout << "Particles (in order): ";
		print(getParticles().begin(), getParticles().end());
	}

	void Transition() {
		assert(false);
	}

	void Likelihood() {
		assert(false);
	}

private:
	int particle_count;
};


void test_filter() {
	TestParticleFilter filter;
	filter.Init();
	filter.Print();
	filter.Resample();
	filter.Print();
}
