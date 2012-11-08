/**
 * @brief 
 * @file File.hpp
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


#ifndef FILE_HPP_
#define FILE_HPP_

#include <string>
#include <vector>

//! The fastest way to implement is to use boost::filesystem::recursive_directory,
//! however, that adds another dependency (boost libraries), so we just use a simple path
//! iterator: http://bit.ly/MYDCno
#include <dirent.h>

#ifdef WARNING
#include <iostream>
#endif

#ifdef VERBOSE
#include <iostream>
#endif

namespace dobots {

/**
 * Separate function to get the filenames in a given path without using third-party libraries,
 * such as boost.
 * @param names				vector with file names that need to be filled (it will not be emptied)
 * @param path				path where we need to search for the given filenames
 * @param substring			substring, e.g. file extension that is used as search mask
 * @param at_end			boolean indicating that the substring needs to be at the end (e.g. file extension)
 * @return success			false on non-existing path (for example)
 */
bool getFilenames(std::vector<std::string> &names, const std::string & path, std::string substring, bool at_end=false) {
	DIR *dp;
	struct dirent *ep;
	dp = opendir(path.c_str());
	if (!dp) {
#ifdef WARNING
		std::cerr << "Something wrong with the working path: " << path << std::endl;
#endif
		return false;
	}
	while ((ep = readdir (dp)) != 0) {
		std::string file = std::string(ep->d_name);
#ifdef VERBOSE
		std::cout << "Found file: " << file << std::endl;
#endif
		std::size_t pos = file.rfind(substring);
		if (pos == std::string::npos) {
#ifdef VERBOSE
			std::cerr << "File " << file << " does not contain substring \"" << substring << "\"" << std::endl;
#endif
			continue;
		}
		if (at_end) {
			if ((pos + substring.size()) == file.size())
				names.push_back(file);
		} else {
			names.push_back(file);
		}
	}
	closedir(dp);
	return true;
}

}

#endif /* FILE_HPP_ */
