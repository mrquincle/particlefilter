/**
 * @brief 
 * @file main.cpp
 *
 * This file is created at Almende B.V. It is open-source software and part of the Common 
 * Hybrid Agent Platform (CHAP). A toolbox with a lot of open-source tools, ranging from 
 * thread pools and TCP/IP components to control architectures and learning algorithms. 
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless,
 * we personally strongly object against this software being used by the military, in the
 * bio-industry, for animal experimentation, or anything that violates the Universal
 * Declaration of Human Rights.
 *
 * Copyright © 2012 Anne van Rossum <anne@almende.com>
 *
 * @author  Anne C. van Rossum
 * @date    Jul 25, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
 */

#include <PositionParticleFilter.h>
#include <Histogram.h>
#include <Container.hpp>
#include <Autoregression.hpp>

#include <algorithm>
#include <vector>

#include <CImg.h>
#include <ImageSource.h>

#include <testRegression.h>
#include <testFilter.h>
#include <createTrackImage.h>

using namespace cimg_library;
using namespace std;

/* **************************************************************************************
 * Implementation of main
 * **************************************************************************************/

template <typename T>
void getHistogram(CImg<T> &img, NormalizedHistogramValues & result) {
	int width = img._width;
	int height = img._height;
	int bins = 16;

	cout << "Create histogram with " << bins << " bins" << endl;
	Histogram histogram(bins, width, height);
	DataFrames frames;
	pDataMatrix data = img._data;
	frames.push_back(data);

	cout << "Add data for histograms" << endl;
	histogram.calcProbabilities(frames);

	// get actual histogram
	histogram.getProbabilities(result);
}

/**
 * Showcase for a particle filter. It - for that reason - uses images, because it is easier
 * to tell what is going on using visuals. The user needs to select the thing to be tracked
 * and the particle filter will subsequently try to track it.
 */
int main() {
//	test_histogram();
//	test_autoregression();
//	test_filter();
//	create_track_image();
//	return EXIT_SUCCESS;

	PositionParticleFilter filter;
	ImageSource source;

	string path = "/home/anne/mydata/active_wheel_camera";
	string extension = ".jpg";

	source.SetPath(path);
	source.SetExtension(extension);
	if (!source.Update()) {
		cerr << "Wrong path?" << endl;
		return EXIT_FAILURE;
	}

	string fn = "target_t1_1924674796";

	string track = fn + ".jpeg";
	CImg<unsigned char> &track_img = *source.getImage<unsigned char>(track);

	NormalizedHistogramValues result;
	getHistogram(track_img, result);

	string config = path + '/' + fn + ".ini";
	cout << "Load config file: " << config << endl;
	ConfigFile configfile(config);
	CImg <int> img_coords(6);
	configfile.readInto(img_coords(0), "coord0");
	configfile.readInto(img_coords(1), "coord1");
	configfile.readInto(img_coords(3), "coord3");
	configfile.readInto(img_coords(4), "coord4");

	filter.Init(result, img_coords, 10);

	int frame_count = 2; // 25
	int frame_id = 0;
	while (++frame_id < frame_count) {

		CImg<unsigned char> &img = *source.getImage<unsigned char>();

		CImgDisplay main_disp(img, "Show image");

		usleep(200000);

		filter.Tick(&img);

		delete &img;
	}

	delete &track_img;
}

