/**
 * @brief Helper functions for C++ std containers
 * @file Container.hpp
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

#ifndef CONTAINER_H_
#define CONTAINER_H_

// General files
#include <assert.h>
#include <algorithm>
#include <functional>
#include <numeric>

/****************************************************************************************************
 * Helper functions for containers.
 ***************************************************************************************************/

/**
 * We are working here with containers. However, we made a typedef of the container class, in case
 * you might find use of something else. Hence, we are very "filthy" here and use a macro to
 * set our preference. To other containers have been used, but it should be okay to use a
 * "std::deque" or a "std::list" too.
 */
#define CONTAINERTYPE vector

// Conveniences
#define DATACONTAINER 		std::CONTAINERTYPE<T>
#define INCLUDECONTAINER 	<CONTAINERTYPE>

// The container object to be used
#include INCLUDECONTAINER

// The "dobots" namespace is not really necessary, but will at times prevent clashes with
// standard std functionality.
namespace dobots {

/**
 * For an explanation of the different metrics, see the "distance" function below. This distance
 * function does really calculate a distance between two containers, say two vectors, and is not
 * the std::distance function that just returns the distance between elements with respect to a
 * given iterator.
 */
enum DistanceMetric {
	DM_EUCLIDEAN,
	DM_DOTPRODUCT,
	DM_BATTACHARYYA,
	DM_HELLINGER,
	DM_MANHATTAN,
	DM_CHEBYSHEV,
	DM_TYPES };

/**
 * Somehow "std::max" is only given as a normal template function and not as a derivation of
 * a function object derived from binary_function.
 * Reason: none
 * Background: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2001/n1293.html
 */
template<typename _Tp>
struct max : public std::binary_function<_Tp, _Tp, _Tp> {
	_Tp operator()(const _Tp& __x, const _Tp& __y) const
	{ return std::max(__x, __y); }
};

/**
 * The p=2 norm, element-wise squaring the difference.
 */
template<typename T>
T euclidean(T x, T y) {
	return (x-y)*(x-y);
}

/**
 * The p=1 norm, can be used for the Manhattan (but also the Chebyshev distance).
 */
template<typename T>
T taxicab(const T & x, const T & y) {
	return std::abs(x-y);
}

/**
 * See: http://en.wikipedia.org/wiki/Bhattacharyya_distance
 * The Battacharyya distance is related to the Battacharyya coefficient.
 */
template<typename T>
T battacharyya(T x, T y) {
	return std::sqrt(x*y);
}

/**
 * See: http://en.wikipedia.org/wiki/Hellinger_distance
 * Related to the Battacharyya coefficient.
 */
template<typename T>
T hellinger(T x, T y) {
	T tmp = std::sqrt(x)-std::sqrt(y);
	return tmp*tmp;
}

/**
 * Create a template function which moves container x from or towards y with a learning rate "mu".
 * A positive mu will move "x" away, while a negative mu will move "x" towards "y".
 */
template<typename T>
class op_adjust: std::binary_function<T,T,T> {
	T mu_;
public:
	op_adjust(T mu): mu_(mu) {}
	T operator()(T x, T y) const {
		//fancy: std::multiplies<T>( std::minus<T>(x,y), mu_);
		T result = x + (x-y)*mu_;
		return result;
	}
};

/**
 * Incremental adjustment of a container towards a reference container.
 *   d = d + mu ( ref - d)
 * So:
 *   delta_d = mu ( ref - d)
 * If "ref" is smaller than "d", "delta_d" will be negative: it will make "d" smaller.
 * Note that this function does not make use of the different distance metrics that can be defined.
 * It will use std::minus (normal -sign) to judge the distance over all elements of the container
 * and adjust them in the same fashion by the multiplication factor mu.
 * If "mu" is 1, it will set the "tomove" container equal to the "reference" container.
 * @param tomove			the container to-be-moved
 * @param reference			the reference container towards the movements happens, the "attractor"
 * @param mu				the step size (0 < mu <= 1)
 */
template <typename T>
void increaseDistance(DATACONTAINER & tomove, const DATACONTAINER & reference, T mu) {
	if (reference.size() != tomove.size()) {
		std::cerr << "Container size unequal: " << reference.size() << " vs " << tomove.size() << std::endl;
		assert (reference.size() == tomove.size());
	}
	assert (mu > T(0)); assert (mu <= T(1));
	std::transform(tomove.begin(), tomove.end(), reference.begin(), tomove.begin(),
			op_adjust<T>(mu));

}

/**
 * Incremental adjustment of a container back from a reference container.
 *   d = d - mu ( ref - d)
 * @param tomove			the container to-be-moved
 * @param reference			the reference container that functions as a "repeller"
 * @param mu				the step size (0 < mu <= 1)
 * @return					void
 */
template <typename T>
void decreaseDistance(DATACONTAINER & tomove, const DATACONTAINER & reference, T mu) {
	if (reference.size() != tomove.size()) {
		std::cerr << "Container size unequal: " << reference.size() << " vs " << tomove.size() << std::endl;
		assert (reference.size() == tomove.size());
	}
	assert (mu > T(0)); assert (mu <= T(1));
	std::transform(tomove.begin(), tomove.end(), reference.begin(), tomove.begin(),
			op_adjust<T>(-mu));
}

/**
 * This function tells something about the "distance" between containers, in other words the similarity or
 * dissimilarity. There are currently several metrics implemented:
 *   DM_DOTPRODUCT:			return sum_i { x_i*y_i }
 *   DM_EUCLIDEAN:			return sqrt (sum_i { (x_i-y_i)^2 } )
 *   DM_BATTACHARYYA:		return -ln (sum_i { sqrt (x_i*y_i) } )
 *   DM_HELLINGER:			return sqrt (sum_i { (sqrt(x_i)-sqrt(y_i))^2 } ) * 1/sqrt(2)
 *   DM_CHEBYSHEV:			return max_i abs(x_i-y_i)
 *   DM_MANHATTAN:			return sum_i { abs(x_i-y_i) }
 * It is assumed that the containers are of equal size.
 * @param container0		one container
 * @param container1		another container
 * @param metric			a certain distance metric
 * @return					the distance between the two containers
 */
template <typename T>
T distance(const DATACONTAINER & container0, const DATACONTAINER & container1, DistanceMetric metric) {
	if (container0.size() != container1.size()) {
		std::cerr << "Container size unequal: " << container0.size() << " vs " << container1.size() << std::endl;
		assert (container0.size() == container1.size());
	}
	switch (metric) {
	case DM_DOTPRODUCT:
		return std::inner_product(container0.begin(), container0.end(), container1.begin(), T(0));
	case DM_EUCLIDEAN:
		return std::sqrt(
				std::inner_product(container0.begin(), container0.end(), container1.begin(), T(0), std::plus<T>(), euclidean<T>));
	case DM_BATTACHARYYA:
		return -std::log(
				std::inner_product(container0.begin(), container0.end(), container1.begin(), T(0), std::plus<T>(), battacharyya<T>));
	case DM_HELLINGER:
		return (std::sqrt(
				std::inner_product(container0.begin(), container0.end(), container1.begin(), T(0), std::plus<T>(), hellinger<T>))) /
				std::sqrt(2);
	case DM_CHEBYSHEV:
		return std::inner_product(container0.begin(), container0.end(), container1.begin(), T(0), max<T>(), taxicab<T>);
	case DM_MANHATTAN:
		return std::inner_product(container0.begin(), container0.end(), container1.begin(), T(0), std::plus<T>(), taxicab<T>);
	default:
		std::cerr << "Unknown distance metric" << std::endl;
		return -1;
	}
}

}

#endif /* CONTAINER_H_ */
