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
 * @date    Sep 18, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
 */


#ifndef IMAGESOURCE_H_
#define IMAGESOURCE_H_

// General files
#include <CImg.h>
#include <string>

/* **************************************************************************************
 * Interface of ImageSource
 * **************************************************************************************/

template <typename Image>
class ImageSource {
public:
	//! Constructor ImageSource
	ImageSource(): img_path(""), img_basename("image"), img_extension(".jpeg") {}

	//! Destructor ~ImageSource
	virtual ~ImageSource() {}

	//! Perform functionality that is required to get images
	virtual bool Update() = 0;

	//! Get an image (the next image if there are multiple).
	virtual Image* getImage() = 0;

	//! Get an image but shifted in maximum two directions.
	virtual Image* getImageShifted(int shift_x, int shift_y) = 0;

	//! Get the path
	void SetPath(std::string path) { img_path = path; }

	//! Get the path
	void SetBasename(std::string basename) { img_basename = basename; }

	//! Get extension mask
	void SetExtension(std::string extension) { img_extension = extension; }

protected:
	//! Search path for the pictures or path where they will need to be written
	std::string img_path;

	//! Basename for the pictures to be loaded or saved
	std::string img_basename;

	//! Mask for the extensions of the pictures to be found or stored
	std::string img_extension;

};

#endif /* IMAGESOURCE_H_ */
