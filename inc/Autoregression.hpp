/**
 * @brief 
 * @file Autoregression.hpp
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
 * @date    Aug 8, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
 */


#ifndef AUTOREGRESSION_HPP_
#define AUTOREGRESSION_HPP_

#include <Container.hpp>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

namespace dobots {

enum RotateDirection {
	RD_LEFT,
	RD_RIGHT,
	RD_COUNT
};

/**
 * The seed used for autoregression. Know your seeds, it makes it easier to repeat experiments.
 */
static int autoregression_seed = 334340;

/**
 * Predict the next value using auto-regression (AR).
 * See also: http://en.wikipedia.org/wiki/Autoregressive_model which describes an autoregressive
 * model also as an all-pole infinite impulse response filter with some additional interpretation.
 * There are no constraints enforced on the coefficients, so the model is not guaranteed to be
 * stationary. For example, for AR(1) the coefficient should be: |\phi| < 1
 * The size of the autoregression is derived from the size of the coefficient container. For now
 * it is expected that the data container is of the same size.
 * @param container			input values x[t-1], x[t-2], x[t-3], ...
 * @param coefficient		parameters of the autoregressive model phi[i], phi[2], phi[3], ...
 * @param variance			(optional) an AR progress has white noise with variance \sigma^2.
 * @param constant			(optional) a constant as in the definition
 * @return 					next value x[t]
 */
template<typename InputIterator1, typename InputIterator2, typename T>
inline T predict(InputIterator1 first1, InputIterator1 last1,
		InputIterator2 first2, T constant, T variance = T(1)) {
	__glibcxx_function_requires(_InputIteratorConcept<InputIterator1>);
	__glibcxx_function_requires(_InputIteratorConcept<InputIterator2>);
	__glibcxx_requires_valid_range(first1, last1);

	//	assert (container.size() == coefficients.size());
	static boost::mt19937 random_number_generator(autoregression_seed);
	boost::normal_distribution<> normal_dist(0.0, variance);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > epsilon(
			random_number_generator, normal_dist);

	T sum = std::inner_product(first1, last1, first2, T(0));
	T prediction = constant + sum + epsilon();
#ifdef VERBOSE
	std::cout << __func__ << ": result is " << prediction << std::endl;
#endif
	return prediction;
}

/**
 * A vector is not the best format to implement a circular buffer. However, sometimes a
 * vector is required for other purposes and then an "advance" method is interesting, it
 * will move all elements in the vector to the front and put the front at the back.
 */
template<typename InputIterator>
void rotate(InputIterator first, InputIterator last, RotateDirection direction = RD_RIGHT) {
	__glibcxx_function_requires(_InputIteratorConcept<InputIterator>);
	__glibcxx_requires_valid_range(first, last);
	if (last - first == 1) return;
	switch (direction) {
	case RD_LEFT:
		std::rotate(first, first+1, last);
		break;
	case RD_RIGHT: default:
		std::rotate(first, last-1, last);
		break;
	}
}

/**
 * Pushes an item upon a container and pops off the oldest value. In case the container is build
 * up out of pointers to objects, make sure you delete the item yourself.
 *
 * @param first			iterator to the beginning of the container
 * @param last			iterator to the end of the container
 * @param item			the item to be added to the container
 * @param direction		the direction in which to push and pop (left or right)
 */
template<typename InputIterator, typename T>
void pushpop(InputIterator first, InputIterator last, T item, RotateDirection direction = RD_RIGHT) {
	__glibcxx_function_requires(_InputIteratorConcept<InputIterator>);
	__glibcxx_requires_valid_range(first, last);
	dobots::rotate(first, last, direction);
	// overwrite first item
	*first = item;
}



}

#endif /* AUTOREGRESSION_HPP_ */
