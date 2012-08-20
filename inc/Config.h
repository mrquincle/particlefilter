/**
 * @brief 
 * @file Config.h
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


#ifndef CONFIG_H_
#define CONFIG_H_

/* **************************************************************************************
 * Configuration options
 * **************************************************************************************/

//! Adds a lot of extra checks, turn it off for performance (by setting it to 0)
#define CAREFUL_USAGE 0

/* **************************************************************************************
 * Configuration option implementations
 * **************************************************************************************/

#if CAREFUL_USAGE == 0
#undef CAREFUL_USAGE
#endif

#ifdef CAREFUL_USAGE
#define QUIT_ON_ERROR { assert(false); }
#define QUIT_ON_ERROR_VAL(RETURN) { assert(false); }
#else
#define QUIT_ON_ERROR { return; }
#define QUIT_ON_ERROR_VAL(RETURN) { return (RETURN); }
#endif

#define ASSERT_EQUAL(EXPRESSION_A, EXPRESSION_B) \
	if ((EXPRESSION_A) != (EXPRESSION_B)) \
			std::cerr << __func__ << ": Assert error " << EXPRESSION_A << " != " << EXPRESSION_B << std::endl; \
	assert((EXPRESSION_A) == (EXPRESSION_B));

#define ASSERT_TRUE(EXPRESSION, MESSAGE) \
	if (!EXPRESSION) \
			std::cerr << __func__ << ": Assert error " << MESSAGE << std::endl; \
	assert(EXPRESSION);

/* **************************************************************************************
 * Configuration option includes
 * **************************************************************************************/

#ifdef CAREFUL_USAGE
#include <cassert>
#include <cstdio> // definition of NULL
#endif


#endif /* CONFIG_H_ */
