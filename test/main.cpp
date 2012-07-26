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

using namespace cimg_library;

/* **************************************************************************************
 * Implementation of main
 * **************************************************************************************/

struct Position {
	int x;
	int y;
	int width;
	int height;
	int histogram;
};

class PositionParticleFilter: public ParticleFilter<Position> {
public:
	void Transition() {

	}
};

const unsigned char white[3] = {255,255,255}, red[3]    = {120,50,80},
		yellow[3] = {200,155,0},   green[3]  = {30,200,70};

/**
 * Showcase for a particle filter. It - for that reason - uses images, because it is
 * easiest to tell what is going on, using image processing.
 */
int main() {
	PositionParticleFilter pf;

	std::string file = "/home/anne/mydata/active_wheel_camera/t1.jpg";
	CImg<unsigned char> img(file.c_str());

	CImgDisplay main_disp(img, "Click a point");


	while (!main_disp.is_closed() && !main_disp.is_keyESC()) {
		main_disp.wait();
		if (main_disp.button() && main_disp.mouse_y()>=0) {
			const int y = main_disp.mouse_y();
			//			img.draw_rectangle;
			//			img.draw_rectangle(0,y0,img.width()-1,y0+12,red);
			// from http://www.codeproject.com/answers/55965/How-to-select-an-area-from-displayed-image-using-C.aspx#answer2
			CImg <int> SelectedImageCords = img.get_select(main_disp,2,0);
			CImg <unsigned char> SelectedImage = img.get_crop(SelectedImageCords(0),SelectedImageCords(1),SelectedImageCords(3),SelectedImageCords(4));
			CImgDisplay disp2(SelectedImage,"Selected Image");
			while(!disp2.is_closed() && !disp2.is_keyESC()) {
				disp2.wait();
			}
		}
	}
}

