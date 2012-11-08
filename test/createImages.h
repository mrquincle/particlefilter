/**
 * @brief 
 * @file createTrackImage.h
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


#ifndef CREATEIMAGES_H_
#define CREATEIMAGES_H_

#include <sstream>
#include <CImg.h>

#include <cstdlib> // getenv
#include <sys/stat.h> // mkdir

using namespace std;
using namespace cimg_library;

typedef cimg_library::CImg<DataValue> ImageType;

void create_images() {

	string home = string(getenv("HOME"));
	if (home.empty()) {
		cerr << "Error: no $HOME env. variable" << endl;
		return;
	}

	string datapath = home + "/mydata";
	string path = home + "/mydata/dotty";
	string basename = "image";
	string extension = ".jpg";
	IpcamImageSource<ImageType> source;
	source.SetPath(path);

	struct stat st;
	int status;
	if (stat(datapath.c_str(), &st) != 0) {
		status = mkdir(datapath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		if (status != 0) {
			cerr << "Could not create directory " << datapath << endl;
			exit (EXIT_FAILURE);
		}
	}
	if (stat(path.c_str(), &st) != 0) {
		status = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		if (status != 0) {
			cerr << "Could not create directory " << path << endl;
			exit (EXIT_FAILURE);
		}
	}

	if (!source.Update()) {
		cerr << "Wrong path?" << endl;
		exit (EXIT_FAILURE);
	}

	int frame_count = 40;
	int frame_id = 0;
	while (++frame_id < frame_count) {
		ImageType &img = *source.getImage();

//		stringstream ss; ss.clear(); ss.str("");
//		ss << path << '/' << basename << '_' << frame_id << ".jpeg";
//		img._save_jpeg(NULL, ss.str().c_str(), 100);

		cout << "Save picture: " << frame_id << endl;
	}

}

#endif /* CREATEIMAGES_H_ */
