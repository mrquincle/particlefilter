/**
 * @brief 
 * @file ImageSource.h
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


#ifndef IMAGESOURCE_H_
#define IMAGESOURCE_H_

#include <CImg.h>
#include <string>
#include <vector>

/* **************************************************************************************
 * Interface of ImageSource
 * **************************************************************************************/


class ImageSource {
public:
	//! Constructor ImageSource
	ImageSource();

	//! Destructor ~ImageSource
	virtual ~ImageSource();

	//! Get the path
	void SetPath(std::string path) { this->path = path; }

	//! Get extension mask
	void SetExtension(std::string extension) { this->extension = extension; }

	//! Perform functionality that is required to get images
	bool Update();

	/**
	 * Get an image (the next image if there are multiple).
	 */
	template <typename T>
	cimg_library::CImg<T>* getImage() {
		return getImage<T>(nextFile());
	}

	/**
	 * Get a specific image with given name, should reside in previously set path.
	 */
	template <typename T>
	cimg_library::CImg<T>* getImage(std::string file) {
		file = path + '/' + file;
		cimg_library::CImg<T> *img = new cimg_library::CImg<T>(file.c_str());
		return img;
	}

protected:
	std::string nextFile();
private:
	std::string path;

	std::string extension;

	//! All files with pictures (does not contain path)
	std::vector<std::string> filenames;

	//! Pointer to current file
	int file_ptr;

	//! Use the entire series in reverse (convenient for tracking)
	bool copy_reverse_series;
};

#endif /* IMAGESOURCE_H_ */
