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


#ifndef CREATETRACKIMAGE_H_
#define CREATETRACKIMAGE_H_

#include <sstream>
#include <CImg.h>
#include <ConfigFile.hpp>

#include <cstdlib> // getenv

using namespace std;
using namespace cimg_library;

void create_track_image() {

	string home = string(getenv("HOME"));
	if (home.empty()) {
		cerr << "Error: no $HOME env. variable" << endl;
		return;
	}

	string path = home + "/mydata/dotty";
	string basename = "image1";
	string extension = ".jpg";

	string file = path + '/' + basename + extension;


	CImg<unsigned char> img(file.c_str());

	CImgDisplay main_disp(img, "Click a point");

	srand(time(NULL));

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

			// show the selection
			CImgDisplay disp2(img_selection, "Cropped image selection");
			while(!disp2.is_closed() && !disp2.is_keyESC()) {
				disp2.wait();

				if (disp2.is_keyS()) {
					int random_id = rand();
					cout << "Save selection with random id: " << random_id << endl;
					stringstream ss; ss.clear(); ss.str("");
					// on purpose .jpeg instead of .jpg, makes it easy to distinguish source images from selected target images
					ss << path << "/target_" << basename << '_' << random_id << ".jpeg";
					img_selection._save_jpeg(NULL, ss.str().c_str(), 100);

					// also store coordinates!
					ss.clear(); ss.str("");
					ss << path << "/target_" << basename << '_' << random_id << ".ini";
					string config_filename = ss.str();
					cout << "Config file: " << config_filename << endl;
					std::ofstream out( config_filename.c_str() );
					if (!out) {
						cerr << "Cannot create config file" << endl;
					} else {
						try {
							ConfigFile configfile(config_filename);
							configfile.add("coord0", img_coords(0));
							configfile.add("coord1", img_coords(1));
							configfile.add("coord3", img_coords(3));
							configfile.add("coord4", img_coords(4));
							out << configfile;
						} catch (const ConfigFile::file_not_found &e) {
							cerr << "Cannot find file: " << config_filename << endl;
						}
						out.close();
					}
				}
			}
		}
	}

}

#endif /* CREATETRACKIMAGE_H_ */
