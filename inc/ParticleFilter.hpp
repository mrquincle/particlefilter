/**
 * @brief Implementation of a particle filter
 * @file ParticleFilter.hpp
 *
 * This file is created at Almende B.V. It is open-source software and part of the Common 
 * Hybrid Agent Platform (CHAP). A toolbox with a lot of open-source tools, ranging from 
 * thread pools and TCP/IP components to control architectures and learning algorithms. 
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless,
 * we personally strongly object against this software being used by the military, in the
 * bio-industry, for animal experimentation, or anything that violates the Universal
 * Declaration of Human Rights.
 *
 * Copyright © 2012 Anne van Rossum <anne@almende.com>
 *
 * @author  Anne C. van Rossum
 * @date    Jul 24, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
 */

#ifndef PARTICLEFILTER_HPP_
#define PARTICLEFILTER_HPP_

// General files
#include <vector>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <iterator>
#include <numeric>
#include <cassert>
#include <cmath>

/* **************************************************************************************
 * Interface of ParticleFilter
 * **************************************************************************************/


/**
 * Local class that knows that colons can be treated as white spaces. It is used by the input stream.
 */
struct colonsep: std::ctype<char> {
	colonsep(): std::ctype<char>(get_table()) {}
	static std::ctype_base::mask const* get_table() {
		static std::vector<std::ctype_base::mask>
		rc(std::ctype<char>::table_size,std::ctype_base::mask());
		rc[':'] = std::ctype_base::space;
		rc[' '] = std::ctype_base::space;
		rc['\n'] = std::ctype_base::space;
		return &rc[0];
	}
};

/**
 * Local class that knows that commas can be treated as white spaces. It is by the input stream
 * operator.
 */
struct commasep: std::ctype<char> {
	commasep(): std::ctype<char>(get_table()) {}
	static std::ctype_base::mask const* get_table() {
		static std::vector<std::ctype_base::mask>
		rc(std::ctype<char>::table_size,std::ctype_base::mask());
		rc[','] = std::ctype_base::space;
		return &rc[0];
	}
};

/**
 * A particle in a particle filter is an entity that has state information, such as the
 * x-position and y-position of your cursor. Or the x,y coordinates plus the size of a
 * football player on your TV screen. Or the x,y location of a robot that moves through
 * an environment and needs to track its own position. It is anything that can be
 * estimated by a cloud of particles and for which this estimation might improve over
 * time my more (although noisy) measurements.
 */
template <typename State>
class Particle {
public:
	Particle() {
		state = new State();
		weight = 0;
	}

	Particle(State *state, double weight) {
		this->state = state;
		this->weight = weight;
	}

	~Particle() {
		weight = 0;
		delete state;
	}

	inline State* getState() { return state; }

	inline double getWeight() { return weight; }

	inline void setState(State* state) { this->state = state; }

	inline void setWeight(double weight) { this->weight = weight; }

	/**
	 * The clone function should not be really necessary, by implementing an assignment operator, a
	 * copy constructor, and a swap function. However, I couldn't get it made to work.
	 */
	Particle *clone() {
		State *s = new State(*state);
		Particle *p = new Particle(s, 0);
		return p;
	}

#ifdef WORKS
	void swap(Particle &p) {
		std::swap(state, p.state);
		std::swap(weight, p.weight);
	}

	Particle & operator=(Particle other) {
		std::cout << __func__ << ": Create particle from another using swap " << std::endl;
		swap(other);
		return *this;
	}
#endif

private:
	State *state;
	double weight;

	friend std::ostream& operator<<(std::ostream& os, const Particle & p) {
		os << p.weight << ',' << p->state;
		return os;
	}

	// append
	friend std::istream& operator>>( std::istream& is, Particle& p) {
		is.imbue(std::locale(std::locale(), new commasep));
		is >> p.weight >> p->state;
		return is;
	}
};


/**
 * Helper function for sorting (with highest weight first)
 */
template <typename State>
bool comp_particles(Particle<State> * p0, Particle<State> * p1) {
	return (p0->getWeight() > p1->getWeight());
}

/**
 * Helper function for summing up and normalizing
 */
template <typename State>
double sum_particle_weight(double sum, Particle<State> * p) {
	return sum + p->getWeight();
}

/**
 * Helper function for normalizing.
 */
template <typename State>
class divide_weight {
public:
	divide_weight(double factor): factor(factor) {}
	void operator()(Particle<State> *p) const {
		p->setWeight(p->getWeight() / factor);
	}
private:
	double factor;
};

template <typename State>
class ParticleFilter;

/**
 * Add particles to a set, and get them out.
 */
template <typename State>
class ParticleSet {
public:
	ParticleSet() { particles.clear(); }

	~ParticleSet() { }

	// Normalize such that total weight sums up to one
	void Normalize() {
		double w = std::accumulate(particles.begin(), particles.end(), double(0), sum_particle_weight<State>);
		std::for_each(particles.begin(), particles.end(), divide_weight<State>(w) );
#ifdef DEBUG
		double check = std::accumulate(particles.begin(), particles.end(), double(0), sum_particle_weight<State>);
		std::cout << "Should now sum to 1: " << check << std::endl;
#endif
	}

protected:

private:
	std::vector<Particle<State>* > particles;

	friend class ParticleFilter<State>;

	// extract
	friend std::ostream& operator<<(std::ostream& os, const ParticleSet & set) {
		std::copy(set.particles.begin(), set.particles.end(), std::ostream_iterator<Particle< State> >(os, ','));
		return os;
	}

	// append
	friend std::istream& operator>>( std::istream& is, ParticleSet& set) {
		is.imbue(std::locale(std::locale(), new colonsep));
		Particle<State> *particle = new Particle<State>();
		while(is >> particle) {
			set.particles.push_down(particle);
			particle = new Particle<State>();
		}
		delete particle;
		return is;
	}
};


/**
 * The particle filter.
 *
 * It starts with an initial (so-called prior) distribution. This can for example be
 * initiated by the user, pointing to the current position of the football player. It
 * can be an exhaustive scan over an entire image to detect a target, which subsequently
 * needs to be tracked, or it can be a random spot (which defines the origin of the
 * coordinate system in a robot which tracks itself).
 *
 * Then the filter needs a transition model. This is an educated guess on how the thing
 * that is tracked changes from state. Something on an image stays which a high
 * probability near the location where it was at the previous time step. Moreover, most
 * likely it moves in the same direction in which it moved before. Of course, this
 * educated guess might be something else than the modeler's knowledge of the thing to
 * be tracked. The transition model might be obtained by a supervised learning process.
 *
 * Third, the filter needs an observation model. This is the probability that that
 * what we want to track is actually at the position that we are consider. So, this is
 * p(object|location), where we use "location" instead of the more general "state" for
 * the story. One way to do this in an image is to calculate the difference between the
 * two histograms, the reference histogram of the object, and the histogram at the
 * location into consideration. Note that if you use a supervised learning method for
 * the transition model, you'll probably need this observation model. However, it is
 * also not hard to imagine that the observation model on a robot is computationally
 * cheap, while for learning the transition model you will use something complex, so
 * you don't need to think about motion equations yourself.
 *
 * In the end we want to have p(location|object), the object that we saw at the start,
 * we want to be able to track over time till the very end of times...
 */
template <typename State>
class ParticleFilter {
public:
	//! Constructor ParticleFilter
	ParticleFilter() {}

	//! Destructor ~ParticleFilter
	virtual ~ParticleFilter() {}

	//! The actual smart part of the particle filter
	void Resample() {
		set.Normalize();
		// sort, with highest weight first
		std::sort(set.particles.begin(), set.particles.end(), comp_particles<State>);
		// now we append
		int N = set.particles.size(); int newN = 0;
		for (int i = 0; i < N; ++i) {
			int copies = round(set.particles[i]->getWeight() * N);
			for (int j = 0; j < copies; ++j) {
				Particle<State> *newp = set.particles[i]->clone();
				set.particles.push_back(newp);
				newN++;
				if (newN == N) {
					// now remove old items
					set.particles.erase(set.particles.begin(), set.particles.begin()+N);
					assert ( set.particles.size() == newN);
					return;
				}
			}
		}
		while (newN < N) {
			// duplicate particle with highest weight to get exactly same number again
			Particle<State> *newp = set.particles[0]->clone();
			set.particles.push_back(newp);
			newN++;
		}
		set.particles.erase(set.particles.begin(), set.particles.begin()+N);
		assert ( set.particles.size() == newN);
	}

	//! Transition according to a certain model
	virtual void Transition() = 0;

	//! Observation model:
	//! - given a particle, how likely that it is corresponding to the tracked entity?
	//! This function should calculate this for all particles and update weights accordingly
	virtual void Likelihood() = 0;

protected:
	//! Hand over access to particles to subclasses
	std::vector<Particle<State>* >& getParticles() { return set.particles; }

private:

	//! The actual cloud of particles
	ParticleSet<State> set;
};

#endif /* PARTICLEFILTER_HPP_ */
