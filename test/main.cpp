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

#include <ParticleFilter.hpp>
#include <CImg.h>

#include <Histogram.h>
#include <Container.hpp>
#include <Autoregression.hpp>
// The alphanum sorting functionality is only added for testing purposes
#include <alphanum.hpp>
#include <algorithm>

//! The fastest way to implement is to use boost::filesystem::recursive_directory,
//! however, that adds another dependency (boost libraries), so we just use a simple path
//! iterator: http://bit.ly/MYDCno
#include <dirent.h>

using namespace cimg_library;
using namespace std;

/* **************************************************************************************
 * Implementation of main
 * **************************************************************************************/

struct ParticleState {
//	int x;
//	int y;
	int width;
	int height;
	int histogram;

	std::vector<Value> x;
	std::vector<Value> y;
	// History x[n-1], x[n-2], etc.
//	int x1, x2;
//	int y1, y2;
};

/**
 * The particle filter that is used for tracking a 2D screen "position" plus some
 * additional state "elaborations", such as width, height, and histogram.
 */
class PositionParticleFilter: public ParticleFilter<ParticleState> {
public:
	PositionParticleFilter() {
		bins = 16;
		seed = 234789;
		auto_coeff.clear();
		auto_coeff.push_back(0.23);
		auto_coeff.push_back(0.12);
		srand48(seed);
	}

	void SetImage(CImg<DataValue> *img) { this->img = img; }

	//! Transition of all particles following a certain motion model
	void Transition() {


	}

	ParticleState *Transition(ParticleState oldp) {
		ParticleState newp;
//		int x = a1 * oldp.x1 + a2 * oldp.x2 + drand48();

		int xn = dobots::predict(oldp.x, auto_coeff);
//		newp.x = std::max(0, std::min((int)img->_width-1, x));
//		newp.y = std::max(0, std::min((int)img->_height-1, y));

	}

	void Likelihood() {
		// for every particle calculate likelihood and "calculate" weight
	}

	float Likelihood(ParticleState state) {
		CImg <unsigned char> img_selection = img->get_crop(
				state.x[0],state.y[0],state.x[0]+state.width,state.y[0]+state.height);
		DataFrames frames;
		pDataMatrix data = img_selection._data;
		frames.push_back(data);

		Histogram histogram(bins, img->_width, img->_height);
		cout << "Add data for histograms" << endl;
		histogram.calcProbabilities(frames);

		NormalizedHistogramValues result;
		histogram.getProbabilities(result);

		Value dist = dobots::distance<Value>(tracked_object_histogram, result, dobots::DM_HELLINGER);
		return dist;
	}
protected:

private:
	//! The number of bins
	int bins;

	//! The histogram of the object to be tracked
	NormalizedHistogramValues tracked_object_histogram;

	//! Image data
	pDataMatrix data;

	//! Image to get data from
	CImg<DataValue> * img;

	//! Seed for random number generator
	int seed;

	//! See http://demonstrations.wolfram.com/AutoRegressiveSimulationSecondOrder/
	std::vector<Value> auto_coeff;


};


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

void test_histogram() {
	cout << " === start test histogram === " << endl;
	int size = 10;
	int bins = 4;
	Histogram histogram(bins, size, 1);

	DataFrames frames;
	int nof_frames = 1;
	for (int f = 0; f < nof_frames; ++f) {
		pDataMatrix data = new DataValue[size];
		for (int i = 0; i < size; ++i) {
			data[i] = 40 * ((i % 2) + 1); // set values 40 or 80
		}
		frames.push_back(data);
	}

	histogram.calcProbabilities(frames);

	std::vector<Value> result;
	histogram.getProbabilities(result);

	cout << "Result: ";
	for (int i = 0; i < result.size(); ++i) {
		cout << result[i] << ' ';
	}
	cout << endl;
	cout << " === end test histogram === " << endl;
}

/**
 * Showcase for a particle filter. It - for that reason - uses images, because it is easier
 * to tell what is going on using visuals. The user needs to select the thing to be tracked
 * and the particle filter will subsequently try to track it.
 */
int main() {
//	test_histogram();
//	return EXIT_SUCCESS;

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
				//			img.draw_rectangle;
				//			img.draw_rectangle(0,y0,img.width()-1,y0+12,red);
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
				}
				//const CImg<float> img_hist = image_selected.histogram(16);
				//img2.display_graph(0,3);

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

