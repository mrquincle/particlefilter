///**
// * @brief
// * @file FileImageSource.cpp
// *
// * This file is created at Almende B.V. It is open-source software and part of the Common
// * Hybrid Agent Platform (CHAP). A toolbox with a lot of open-source tools, ranging from
// * thread pools and TCP/IP components to control architectures and learning algorithms.
// * This software is published under the GNU Lesser General Public license (LGPL).
// *
// * It is not possible to add usage restrictions to an open-source license. Nevertheless,
// * we personally strongly object to this software being used by the military, in factory
// * farming, for animal experimentation, or anything that violates the Universal
// * Declaration of Human Rights.
// *
// * Copyright © 2012 Anne van Rossum <anne@almende.com>
// *
// * @author  Anne C. van Rossum
// * @date    Aug 15, 2012
// * @project Replicator FP7
// * @company Almende B.V.
// * @case    modular robotics / sensor fusion
// */
//
//#include <FileImageSource.h>
//#include <File.hpp>
//#include <alphanum.hpp>
//#include <algorithm>
//#include <iostream>
//
//#include <Config.h>
//
//using namespace std;
//using namespace dobots;
////using namespace cimg_library;
//
///* **************************************************************************************
// * Implementation of FileImageSource
// * **************************************************************************************/
//
///**
// * Update, for example if a directory contents has changed.
// */
//template <typename Image>
//bool FileImageSource<Image>::Update() {
//	// clear history
//	filenames.clear();
//
//	// get all *.jpg files
//	string extension = ".jpg";
//	bool success = getFilenames(filenames, path, extension, true);
//	if (!success) QUIT_ON_ERROR_VAL(false);
//
//	if (filenames.empty()) {
//		cerr << "No pictures available!" << endl;
//		return false;
//	}
//
//	// sort in such order that the one with the lowest "postfix" comes first (t1.jpg ... t10.jpg)
//	sort(filenames.begin(), filenames.end(), doj::alphanum_less<string>());
//
//	// set pointer to first file
//	file_ptr = 0;
//
//	// make sure looping through the images allows for "continuous" tracking by adding the entire
//	// sequence in reverse [0, 1, 2, 3] becomes [0, 1, 2, 3, 2, 1] upon which can be looped
//	if (copy_reverse_series) {
//		size_t cnt = filenames.size();
//		if (cnt > 2) {
//			for (size_t i = cnt-2; i > 0; --i) {
//				filenames.push_back(filenames[i]);
//			}
//			assert (filenames.size() == (cnt-1)*2);
//		}
//	}
//
//	return success;
//}
//
//template <typename Image>
//std::string  FileImageSource<Image>::nextFile() {
//	if (file_ptr < 0) {
//		std::cerr << "Call to update() was probably not successful" << std::endl;
//		QUIT_ON_ERROR_VAL("");
//	}
//	assert (file_ptr < filenames.size());
//	std::string file = filenames[file_ptr];
//	file_ptr = (file_ptr + 1) % filenames.size();
//	std::cout << "Open file " << file << std::endl;
//	return file;
//}
