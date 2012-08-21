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

#include <testDistance.h>
#include <testRegression.h>
#include <testHistogram.h>
#include <testFilter.h>
#include <createTrackImage.h>

using namespace cimg_library;
using namespace std;

const DataValue red[] = { 255,0,0 }, green[] = { 0,255,0 }, blue[] = { 0,0,255 };

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

static int seed = 239483;

/**
 * Showcase for a particle filter. It - for that reason - uses images, because it is easier
 * to tell what is going on using visuals. The user needs to select the thing to be tracked
 * and the particle filter will subsequently try to track it.
 */
int main() {
//	test_histogram();
//	test_autoregression();
//	test_filter();
//	test_distance();
//	create_track_image();
//	return EXIT_SUCCESS;

	PositionParticleFilter filter;
	ImageSource source;

	string path = "/home/anne/mydata/active_wheel_camera";
	string extension = ".jpg";

	srand48(seed);

	source.SetPath(path);
	source.SetExtension(extension);
	if (!source.Update()) {
		cerr << "Wrong path?" << endl;
		return EXIT_FAILURE;
	}

	string fn = "target_t1_1924674796";

	string track = fn + ".jpeg";
	CImg<DataValue> &track_img = *source.getImage<DataValue>(track);

	NormalizedHistogramValues result;
	getHistogram(track_img, result);

	string config = path + '/' + fn + ".ini";
	cout << "Load config file: " << config << endl;
	ConfigFile configfile(config);
	CImg <CoordValue> img_coords(6);
	configfile.readInto(img_coords(0), "coord0");
	configfile.readInto(img_coords(1), "coord1");
	configfile.readInto(img_coords(3), "coord3");
	configfile.readInto(img_coords(4), "coord4");

	int subticks = 1;
	int particles = 100;
	int shift = 4;
	filter.Init(result, img_coords, particles);

	vector <CImg<CoordValue>*> coordinates;

	int frame_count = 40;
	int frame_id = 0;
	while (++frame_id < frame_count) {

//		CImg<DataValue> &img = *source.getImage<DataValue>();
		CImg<DataValue> &img = *source.getImageShifted<DataValue>(frame_id*shift, 0);

		cout << "Clear coordinates" << endl;
		coordinates.erase(coordinates.begin(), coordinates.end());
		coordinates.clear();
		filter.GetParticleCoordinates(coordinates);

		CImg<DataValue> &img_copy = *new CImg<DataValue>(img);
		int max = 10;
		for (int i = 0; i < coordinates.size(); ++i) {
			CImg<CoordValue>* coord = coordinates[i];
			//cout << "Draw rectangle at: [" << coord->_data[0] << "," << coord->_data[1] << "," << coord->_data[3] << "," << coord->_data[4] << "]" << endl;
			float opacity = 0.05;
			img_copy.draw_line(coord->_data[0], coord->_data[1], coord->_data[0], coord->_data[4], red);
			img_copy.draw_line(coord->_data[0], coord->_data[1], coord->_data[3], coord->_data[1], red);
			img_copy.draw_line(coord->_data[3], coord->_data[1], coord->_data[3], coord->_data[4], red);
			img_copy.draw_line(coord->_data[0], coord->_data[4], coord->_data[3], coord->_data[4], red);
			if (--max == 0) break;
		}

		CImgDisplay main_disp(img_copy, "Show image");

//		usleep(2000000);

		main_disp.wait(2000);
		if (main_disp.is_keyESC()) { // doesn't work, is not captured during wait (is usleep)
			delete &img;
			cout << "Escape by user, exit" << endl;
			break;
		}

		cout << "Particle filter tick " << frame_id << endl;
		filter.Tick(&img, subticks);

#ifdef DISPLAY_LIKELIHOOD
		cout << "Calculate likelihood for all pixels" << endl;
		RegionSize region;
		region.width = img_coords(3)-img_coords(0);
		region.height = img_coords(4)-img_coords(1);
		cout << "Create picture of size " << img._height*img._width << endl;

		CImg<DataValue> &img2 = *new CImg<DataValue>(img._width, img._height, 1, 3);
		filter.GetLikelihoods(img2, region);

		CImgDisplay disp(img2, "Show values");

		usleep(4000000);
		delete &img2;
#endif

		delete &img;
	}

	delete &track_img;
}

