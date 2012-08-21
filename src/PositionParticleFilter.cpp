/**
 * @brief 
 * @file PositionParticleFilter.cpp
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
 * @date    Aug 14, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
 */
#include <PositionParticleFilter.h>

using namespace dobots;

#include <Print.hpp>

/* **************************************************************************************
 * Implementation of PositionParticleFilter
 * **************************************************************************************/

PositionParticleFilter::PositionParticleFilter() {
	bins = 16;
	seed = 234789;
	auto_coeff.clear();
	auto_coeff.push_back(2.0);
	auto_coeff.push_back(-1.0);
	srand48(seed);
	img = NULL;
}

PositionParticleFilter::~PositionParticleFilter() {

}

/**
 * Do every action that is necessary to update all particles. This takes three steps:
 * - transition according to a certain motion model
 * - observing the likelihood of the object being at the translated position (results in a weight)
 * - resample according to that likelihood (given by the weight)
 * @param img_frame			the image with the entitie(s) to be tracked
 * @param subticks			the number of times this same image needs to be used
 */
void PositionParticleFilter::Tick(CImg<DataValue> *img_frame, int subticks)  {
	img = img_frame;
	assert (subticks > 0);
	for (int i = 0; i < subticks; ++i) {
		cout << "Transition all particles" << endl;
		Transition();
		cout << "Likelihood for all particles" << endl;
		Likelihood();
		cout << "Resample all particles" << endl;
		Resample();
	}
}

/**
 * Initialise the particle filter.
 */
void PositionParticleFilter::Init(NormalizedHistogramValues &tracked_object_histogram,
		CImg<CoordValue> &coord, int particle_count) {

	getParticles().clear();

	int width = coord(3) - coord(0);
	int height = coord(4) - coord(1);

	cout << "Width*height=" << width << '*' << height << endl;

	this->tracked_object_histogram = tracked_object_histogram;

	// generate duplicates of particles
	for (int i = 0; i < particle_count; ++i) {

		ParticleState *s = new ParticleState(i+1);
		s->width = width;
		s->height = height;
		s->x.clear();
		s->y.clear();
		int history_size = 2;
		for (int j = 0; j < history_size; ++j) {
			s->x.push_back(coord(0) + width / 2);
			s->y.push_back(coord(1) + height / 2);
			s->scale.push_back(1);
		}
		Particle<ParticleState> *p  = new Particle<ParticleState>(s, 0);
		//	s. scale/ histogram
		getParticles().push_back(p);
	}

	ASSERT_EQUAL(getParticles().size(), particle_count);
}

/**
 * Transition of all particles following a certain motion model. Due to the fact there is
 * no multiplication or removal in the number of particles at this moment, we can safely
 * iterate through the entire container and do the transition "in place".
 */
void PositionParticleFilter::Transition() {
	std::vector<Particle<ParticleState>* >::iterator i;
	for (i = getParticles().begin(); i != getParticles().end(); ++i) {
		ParticleState *state = (*i)->getState();
		assert (state != NULL);
		Transition(*state);
	}

//	ASSERT_EQUAL(getParticles().size(), particle_count);
}

void PositionParticleFilter::Likelihood() {
	std::vector<Particle<ParticleState>* >::iterator i;
	for (i = getParticles().begin(); i != getParticles().end(); ++i) {
		ParticleState *state = (*i)->getState();
		assert (state != NULL);
		state->likelihood = Likelihood(*state);
		(*i)->setWeight(state->likelihood);
	}

	// log for the user
	std::sort(getParticles().begin(), getParticles().end(), comp_particles<ParticleState>);
	int max = 10;
	cout << "Likelihoods: ";
	for (i = getParticles().begin(); i != getParticles().end(); ++i) {
		ParticleState *state = (*i)->getState();
		cout << '[' << state->getId() << ':' << state->likelihood << "] ";
		if (--max == 0) break;
	}
	cout << endl;

//	ASSERT_EQUAL(getParticles().size(), particle_count);
}

/**
 * Return the particle coordinates for display.
 */
void PositionParticleFilter::GetParticleCoordinates(std::vector<CImg<CoordValue> *> & coordinates) {
	// sort to make sure the one with highest weight comes first
	std::sort(getParticles().begin(), getParticles().end(), comp_particles<ParticleState>);

	std::vector<Particle<ParticleState>* >::iterator i;
	for (i = getParticles().begin(); i != getParticles().end(); ++i) {
		CImg<CoordValue> *coord = new CImg<CoordValue>(6);
		ParticleState *state = (*i)->getState();
		assert (state != NULL);
		assert (state->getId());
		std::string msg; msg = "state is empty " + state->getId();
		if(state->x.empty()) cout << msg << endl;
		assert (!state->x.empty());
		assert (!state->y.empty());
		assert (!state->scale.empty());
		float x = state->x.front();
		float y = state->y.front();
		float scale = state->scale.front();
		float width = state->width * scale;
		float height = state->height * scale;
		coord->_data[0] = x-width/2;
		coord->_data[1] = y-height/2;
		coord->_data[3] = x+width/2;
		coord->_data[4] = y+height/2;
		coordinates.push_back(coord);
	}
}

/**
 * Normal state of affairs is to use an autoregressive model to estimate where an
 * object will be next. There are however many different autoregressive models in use,
 * and it doesn't seem there is a "standard" way...:
 * <ul>
 * <li>Robust Visual Tracking for Multiple Targets (Cai, Freitas, Little):
 *     x[n] = A x[n-1] + B x[n-2] + C N(0,1) </li>
 * <li>CamShift Guided Particle Filter for Visual Tracking (Wang, Yang, Xy, Yu):
 *     x[n] = 2*x[n-1] - x[n-2] + N(0,1) </li>
 * <li>Real-time Hand Tracking using a Mean Shift Embedded Particle Filter (Shan,
 * Tan, Wei):
 *     (x[n] - x[n-1]) = (x[n-1] - x[n-2]) + N(0,1)</li>
 * <li>PFCHA: A New Moving Object Tracking Algorithms Based on Particle Filter
 * and Histogram (Qi, JiaFu):
 *     (x[n] - x_avg) = A (x[n-1] - x_avg) + B (x[n-2] - x_avg) + N(0,1)</li>
 * </ul>
 * Obviously, the second and third are the same. To have a second-order autoregressive
 * process "over differences" it is for example also possible to use:
 *   x[n+1] = x[n] + A (x[n] - x[n-1]) + B (x[n-1] - x[n-2]) + N(0,1)
 * etcetera, etcetera.
 * I think it's smart to consider a formulation where x[n], x[n-1], ... can be used
 * directly, so we do not need to store differences in our vector for "predict", hence
 * I opt for the second formulation (auto_coeff[0] = 2, auto_coeff[1] = -1).
 *
 */
void PositionParticleFilter::Transition(ParticleState &oldp) {

//#define OVERWRITE

	int xn = dobots::predict(oldp.x.begin(), oldp.x.end(), auto_coeff.begin(), 0.0, 1.0);
	int yn = dobots::predict(oldp.y.begin(), oldp.y.end(), auto_coeff.begin(), 0.0, 1.0);
	Value scale = dobots::predict(oldp.scale.begin(), oldp.scale.end(), auto_coeff.begin(), 0.0, 0.001);

	xn = std::max(0, std::min((int)img->_width-1, xn));
	yn = std::max(0, std::min((int)img->_height-1, yn));
	scale = std::max<Value>(0.1, scale); // scale should not fall below 0.1..

#ifdef OVERWRITE
	xn = oldp.x[0];
	yn = oldp.y[0];
	scale = oldp.scale[0];

//	static boost::mt19937 random_number_generator(autoregression_seed);
//	boost::normal_distribution<> normal_dist(0.0, 1);
//	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > epsilon(
//			random_number_generator, normal_dist);

//	float p = drand48();

	xn += (drand48() * 4 - 2); //epsilon();
	yn += (drand48() * 4 - 2); //epsilon();

#endif
	scale = 1.0;

	dobots::pushpop(oldp.x.begin(), oldp.x.end(), xn);
	dobots::pushpop(oldp.y.begin(), oldp.y.end(), yn);
	dobots::pushpop(oldp.scale.begin(), oldp.scale.end(), scale);

//	cout << "Transition particle " << oldp << endl;
}

/**
 * The likelihood of a player at all locations in the image using a given region size. The result is
 * written back in the form of a picture with colour values.
 * This function takes VERY long if every pixel would have been considered, hence, a block_size
 * parameter exists which only calculates the probability around blocks of this given size.
 * @param result		image with red colour indicating the likelihood, the more black, the likelier
 * @param region_size	the rectangle over which to match the histograms
 * @param block_size	(optional) parameter for subsampling
 * @return				void
 */
void PositionParticleFilter::GetLikelihoods(CImg<DataValue> & result, RegionSize region_size, 	int block_size) {
	assert (img != NULL);

	cout << "Clean entire picture" << endl;
	for (int i = 0; i < result._width; ++i) {
		for (int j = 0; j < result._height; ++j) {
			const DataValue color[] = { 255,255,255 };
			result.draw_point(i,j,color);
		}
	}
	cout << "Calculate likelihood for all pixels (except at distance \"width\" from border)" << endl;
	cout << "  this ranges from " << region_size.width << " to " << result._width-region_size.width << " and ";
	cout << "from " << region_size.height << " to " << result._height-region_size.height << endl;

	ParticleState state;
	state.height = region_size.height;
	state.width = region_size.width;

	bool show_progress = true;
	if (show_progress) cout << endl;
	for (int j = region_size.height; j < result._height-region_size.height; j=j+block_size) {
		if (show_progress) if (!(j%10)) cout << j << ':' << ' ';
		for (int i = region_size.width; i < result._width-region_size.width; i=i+block_size) {
			state.x.clear();
			state.y.clear();
			state.x.push_back(i);
			state.y.push_back(j);
			float value = Likelihood(state);
			DataValue val = value*255;
			if (show_progress) if (!(j%10) && !(i%10)) cout << (int)val << ' ';
			const DataValue color[] = { val,0,0 };
			result.draw_rectangle(i-block_size/2,j-block_size/2,i+block_size/2,j+block_size/2,color);
			if (show_progress) if (!(j%10) && !(i%10)) cout << '+';
		}
		if (show_progress) if (!(j%10)) cout << endl;
	}
}

/**
 * Calculate the likelihood of a player and the state indicated by the parameter
 * "state" which contains an x and y position, a width and a height. This is used
 * to define a rectangle for which a histogram is matched against the reference
 * histogram of the object that is tracked.
 * @param state			the state of the particle (position, width, height)
 * @return				conceptual "distance" to the reference (tracked) object
 */
float PositionParticleFilter::Likelihood(ParticleState & state) {
	assert (img != NULL);
	CImg <CoordValue> coord(6);
	float scale = state.scale.front();
	scale = 1;
	coord._data[0] = state.x[0] - scale * state.width/2;
	coord._data[1] = state.y[0] - scale * state.height/2;
	coord._data[3] = state.x[0] + scale * state.width/2;
	coord._data[4] = state.y[0] + scale * state.height/2;
	CImg <DataValue> img_selection = img->get_crop(coord._data[0], coord._data[1], coord._data[3], coord._data[4]);
	DataFrames frames;
	frames.clear();
	pDataMatrix data = img_selection._data;
	frames.push_back(data);

	Histogram histogram(bins, img_selection._width, img_selection._height);
#ifdef VERBOSE
	cout << __func__ << ": Add data for histograms" << endl;
#endif
	assert (frames.size() == 1);
	histogram.calcProbabilities(frames);

#ifdef VERBOSE
	cout << __func__ << ": Get normalized probabilities" << endl;
#endif
	NormalizedHistogramValues result;
	histogram.getProbabilities(result);

#ifdef VERBOSE
	cout << __func__ << ": Calculate distance to histogram of the to-be-tracked object" << endl;
#endif

	Value dist = dobots::distance<Value>(tracked_object_histogram.begin(), tracked_object_histogram.end(), result.begin(), result.end(),
			dobots::DM_SQUARED_HELLINGER);
	return std::exp(-20.0 * dist);
}

