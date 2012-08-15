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

#include <alphanum.hpp>
#include <algorithm>
#include <vector>

#include <CImg.h>

//! The fastest way to implement is to use boost::filesystem::recursive_directory,
//! however, that adds another dependency (boost libraries), so we just use a simple path
//! iterator: http://bit.ly/MYDCno
#include <dirent.h>

#include <testRegression.h>
#include <testFilter.h>

using namespace cimg_library;
using namespace std;

/* **************************************************************************************
 * Implementation of main
 * **************************************************************************************/

const unsigned char white[3] = {255,255,255},
		red[3] = {120,50,80},
		yellow[3] = {200,155,0},
		green[3] = {30,200,70};


/**
 * Separate functions to get the proper filenames, because this allows sorting afterwards.
 * Return false on errors (for example path does not exist)
 */
bool getFilenames(vector<string> &names, std::string path, std::string extension) {
	DIR *dp;
	struct dirent *ep;
	dp = opendir(path.c_str());
	if (!dp) {
		cerr << "Something wrong with the working path: " << path << endl;
		return false;
	}
	while ((ep = readdir (dp)) != 0) {
		string file = string(ep->d_name);
		if (file.find(extension) == string::npos) {
#ifdef VERBOSE
			cerr << "File " << file << " does not have proper extension \"" << extension << "\"" << endl;
#endif
			continue;
		}
		names.push_back(file);
	}
	closedir(dp);
	return true;
}


/**
 * Showcase for a particle filter. It - for that reason - uses images, because it is easier
 * to tell what is going on using visuals. The user needs to select the thing to be tracked
 * and the particle filter will subsequently try to track it.
 */
int main() {
//	test_histogram();
//	test_autoregression();
	test_filter();
	return EXIT_SUCCESS;

	PositionParticleFilter pf;

	string path = "/home/anne/mydata/active_wheel_camera";
	string extension = ".jpg";

	std::vector<string> filenames;
	filenames.clear();
	bool success = getFilenames(filenames, path, extension);
	if (!success) return EXIT_FAILURE;

	//! Sort in such order that the one with the lowest "postfix" comes first (t1.jpg ... t10.jpg)
	std::sort(filenames.begin(), filenames.end(), doj::alphanum_less<std::string>());

	int frame_id = 0;
	std::vector<string>::iterator it;
	for (it = filenames.begin(); it != filenames.end(); ++it) {

		// Add path to filename
		string file = path + '/' + (*it);
		cout << "Open file " << file << endl;

		CImg<unsigned char> img(file.c_str());

		CImgDisplay main_disp(img, "Click a point");


		while (!main_disp.is_closed() && !main_disp.is_keyESC()) {
			main_disp.wait();
			if (main_disp.button() && main_disp.mouse_y()>=0) {
				const int y = main_disp.mouse_y();
				// img.draw_rectangle;
				// img.draw_rectangle(0,y0,img.width()-1,y0+12,red);
				// from http://www.codeproject.com/answers/55965/How-to-select-an-area-from-displayed-image-using-C.aspx#answer2
				CImg <int> img_coords = img.get_select(main_disp,2,0);
				CImg <unsigned char> img_selection = img.get_crop(img_coords(0),img_coords(1),img_coords(3),img_coords(4));

				cout << "Coordinates: [";
				cout << img_coords(0) << ',' << img_coords(1) << ',' << img_coords(3) << ',' << img_coords(4);
				cout << ']' << endl;

				if (frame_id == 0) {
					int width = img_coords(3) - img_coords(0);
					int height = img_coords(4) - img_coords(1);
					int bins = 16;
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

#ifdef CIMG_HISTOGRAM
					const CImg<float> img_hist = img_selection.histogram(16, 0, 255);
					img_hist.display_graph(0,3);
#endif
				}

				// show the selection
				CImgDisplay disp2(img_selection, "Cropped image selection");
				while(!disp2.is_closed() && !disp2.is_keyESC()) {
					disp2.wait();
				}
			}
		}

		++frame_id;
	}
}

