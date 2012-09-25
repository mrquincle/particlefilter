/**
 * @brief Image source which gets images from files
 * @file FileImageSource.h
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


#ifndef FILEIMAGESOURCE_H_
#define FILEIMAGESOURCE_H_

#include <CImg.h>
#include <string>
#include <vector>
#include <cassert>

#include <File.hpp>
#include <alphanum.hpp>
#include <algorithm>
#include <iostream>

#include <Config.h>

#include <ImageSource.h>

/* **************************************************************************************
 * Interface of FileImageSource
 * **************************************************************************************/

template <typename Image>
class FileImageSource: public ImageSource<Image> {
public:
	//! Constructor FileImageSource
	FileImageSource(): file_ptr(-1), copy_reverse_series(true) {
		filenames.clear();
	}

	//! Destructor ~FileImageSource
	virtual ~FileImageSource() {}

	//! Perform functionality that is required to get images
	bool Update() {
		assert(!this->img_path.empty());

		// clear history
		filenames.clear();

		// get all *.jpg files
		string extension = ".jpg";
		bool success = dobots::getFilenames(filenames, this->img_path, this->img_extension, true);
		if (!success) QUIT_ON_ERROR_VAL(false);

		if (filenames.empty()) {
			cerr << "No pictures available!" << endl;
			return false;
		}

		// sort in such order that the one with the lowest "postfix" comes first (t1.jpg ... t10.jpg)
		sort(filenames.begin(), filenames.end(), doj::alphanum_less<string>());

		// set pointer to first file
		file_ptr = 0;

		// make sure looping through the images allows for "continuous" tracking by adding the entire
		// sequence in reverse [0, 1, 2, 3] becomes [0, 1, 2, 3, 2, 1] upon which can be looped
		if (copy_reverse_series) {
			size_t cnt = filenames.size();
			if (cnt > 2) {
				for (size_t i = cnt-2; i > 0; --i) {
					filenames.push_back(filenames[i]);
				}
				assert (filenames.size() == (cnt-1)*2);
			}
		}

		return success;
	}

	/**
	 * Get an image (the next image if there are multiple).
	 */
	Image* getImage() {
		return getImage(nextFile());
	}

	/**
	 * Get a specific image with given name, should reside in previously set path.
	 * Assumes that there is a constructor that accepts a filename and returns an Image object.
	 */
	Image* getImage(std::string file) {
		file = this->img_path + '/' + file;
		Image *img = new Image(file.c_str());
		return img;
	}

	/**
	 * Gets shifted image. Assumes that there is a shift operator defined on Image!
	 */
	Image* getImageShifted(int shift_x, int shift_y) {
		assert (!filenames.empty());
		Image *img = getImage(filenames.front());
		img->shift(shift_x, shift_y, 0, 0, 2);
		return img;
	}

protected:
	//! Return next file from the previously build up vector with image filenames
	std::string nextFile() {
		if (file_ptr < 0) {
			std::cerr << "Call to update() was probably not successful" << std::endl;
			QUIT_ON_ERROR_VAL("");
		}
		assert (file_ptr < filenames.size());
		std::string file = filenames[file_ptr];
		file_ptr = (file_ptr + 1) % filenames.size();
		std::cout << "Open file " << file << std::endl;
		return file;
	}
private:
	//! All files with pictures (does not contain path)
	std::vector<std::string> filenames;

	//! Pointer to current file
	int file_ptr;

	//! Use the entire series in reverse (convenient for tracking)
	bool copy_reverse_series;
};

#endif /* FILEIMAGESOURCE_H_ */
