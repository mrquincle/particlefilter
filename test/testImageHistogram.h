/**
 * @brief 
 * @file testImageHistogram.h
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


#ifndef TESTIMAGEHISTOGRAM_H_
#define TESTIMAGEHISTOGRAM_H_

#include <CImg.h>
#include <ImageSource.h>

using namespace cimg_library;
using namespace std;

/* **************************************************************************************
 * Interface of testImageHistogram
 * **************************************************************************************/

void test_image_histogram() {

	PositionParticleFilter pf;
	ImageSource source;

	string path = "/home/anne/mydata/active_wheel_camera";
	string extension = ".jpg";

	source.SetPath(path);
	source.SetExtension(extension);
	source.Update();

	CImg<unsigned char> & img = *source.getImage<unsigned char>();

	CImgDisplay main_disp(img, "Show image");

	while (!main_disp.is_closed() && !main_disp.is_keyESC()) {
		main_disp.wait();
		if (main_disp.button() && main_disp.mouse_y()>=0) {
			const int y = main_disp.mouse_y();
			CImg <int> img_coords = img.get_select(main_disp,2,0);
			CImg <unsigned char> img_selection = img.get_crop(img_coords(0),img_coords(1),img_coords(3),img_coords(4));

			cout << "Coordinates: [";
			cout << img_coords(0) << ',' << img_coords(1) << ',' << img_coords(3) << ',' << img_coords(4);
			cout << ']' << endl;

			int width = img_coords(3) - img_coords(0);
			int height = img_coords(4) - img_coords(1);
			int bins = 16;

			cout << "Create histogram with " << bins << " bins" << endl;
			Histogram histogram(bins, width, height);
			DataFrames frames;
			pDataMatrix data = img_selection._data;
			frames.push_back(data);

			cout << "Add data for histograms" << endl;
			histogram.calcProbabilities(frames);

			NormalizedHistogramValues result;
			histogram.getProbabilities(result);

			cout << "Result: ";
			Value sum = 0;
			for (int i = 0; i < result.size(); ++i) {
				sum += result[i];
				cout << result[i] << ' ';
			}
			cout << " with sum = " << sum << endl;

			// Also create histogram from the image using CImg functionality
			// in case [0,2556] are not given as boundaries, these boundaries are dynamically set!
			const CImg<float> img_hist = img_selection.histogram(16, 0, 256);
			img_hist.display_graph(0,3);
		}
	}

	delete &img;
}

#endif /* TESTIMAGEHISTOGRAM_H_ */
