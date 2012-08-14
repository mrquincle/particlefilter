/**
 * @brief Some functions to make printing easier
 * @file Print.hpp
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


#ifndef PRINT_HPP_
#define PRINT_HPP_

#include <string>
#include <sstream>
#include <iostream>
#include <iterator>

namespace dobots {

template<typename InputIterator>
void print(InputIterator first, InputIterator last, std::string delim=", ", std::string empty="[]")
{
	// concept requirements
	__glibcxx_function_requires(_InputIteratorConcept<InputIterator>);
	__glibcxx_requires_valid_range(first, last);

	typedef typename std::iterator_traits<InputIterator>::value_type ValueType;

	if (first == last) {
		std::cout << empty;
		return;
	}
	if (last - first == 1) {
		std::cout << *first;
		return;
	}
	std::ostringstream ss;
	std::copy(first, last - 1, std::ostream_iterator< ValueType >(ss, delim.c_str()));
	ss << *(last - 1);
	std::cout << ss.str() << std::endl;
}

}


#endif /* PRINT_HPP_ */
