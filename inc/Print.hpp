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

/**
 * Function to print to standard out. It will accept any container for which an iterator is defined.
 * The default separator/delimiter is the comma plus a white space. The default empty vector are square
 * brackets. And by default there will be printed an end of line.
 * @param first			First item of the range to be printed
 * @param last			Last item
 * @param delim			A string of symbols separating the items (will not be printed at the end)
 * @param empty			The string denoting an empty container
 * @param endl			Printing a new line at the end of the output or not.
 */
template<typename InputIterator>
void print(InputIterator first, InputIterator last, std::string delim=", ", std::string empty="{}", bool endl=true)
{
	// concept requirements
	__glibcxx_function_requires(_InputIteratorConcept<InputIterator>);
	__glibcxx_requires_valid_range(first, last);
	typedef typename std::iterator_traits<InputIterator>::value_type ValueType;

	typedef typename std::iterator_traits<InputIterator>::difference_type DistanceType;

	DistanceType dist = std::distance(first, last);
	std::cout << '[' << dist << "] ";
	switch (dist) {
	case 0:
		std::cout << empty;
		break;
	case 1:
		std::cout << *first;
		break;
	default:
		std::ostringstream ss;
		std::copy(first, last - 1, std::ostream_iterator< ValueType >(ss, delim.c_str()));
		ss << *(last - 1);
		std::cout << ss.str();
		break;
	}
	if (endl) std::cout << std::endl;
}

}


#endif /* PRINT_HPP_ */
