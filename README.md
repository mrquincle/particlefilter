<!-- Uses markdown syntax for neat display at github -->

# ParticleFilter
A particle filter can be used for tracking:

- to track an object in a sequence of camera images. 
- to track the position of a robot given inaccurate position and movement estimates.

## What does it do?
This software is actually just one file, [ParticleFilter.hpp](https://github.com/mrquincle/particlefilter/blob/master/inc/ParticleFilter.hpp), which contains only one important function, namely "Resample". Every particle stands for an estimate of the actual world or robot state. Each particle can be better or worse representing the world state and this is indicated by one value, its "weight". The weight is determined by an observation model (which is not part of the particle filter itself, but depends on the application). The "Resample" function does basically three things:

- normalizing the particle weights so they sum up to one
- sort the particles according to their weights
- make copies of a particle with the number of copies depending on the weight of the particle (favoring obesity ;-) )

Now, we have a new set of particles and a so-called transition model moves each of these particles according to a predefined model by the user. Due for example to our limited physical capabilities human movements would follow some autoregressive model. An example is given by the [PositionParticleFilter.h](https://github.com/mrquincle/particlefilter/blob/master/inc/PositionParticleFilter.h) that does implement an observation and a transition model.

## Is it good?
Current phase: just out of alpha. And not with respect to the [series](http://en.wikipedia.org/wiki/Alphas) but with respect to the fact that all functionality is there, however everything needs to be debugged.

## What are the alternatives?
There are many alternatives for tracking, done here by particle filtering, ranging from Kalman filters to (other types of) Bayes filters. Then there are many forms of particle filters, for example the boosted particle filter, the unscented particle filter, the cascade particle filter, the kernel particle filter, the bootstrap particle filter and the mean-shift embedded particle filter. And last, but not least, there are software alternatives, such as the particle filter implementation in the [Mobile Robot Programming Toolkit](http://www.mrpt.org/Particle_Filters).

## An example
An example of the particle filter being able to track the docking element of a Replicator robot will follow soon. The following pictures shows the match of all pixels with the reference histogram.

![picture](https://raw.github.com/mrquincle/particlefilter/master/doc/track_robot.jpg)

## Interesting
Maybe you find convenient or interesting some of the helper files that have been written for the particle filter.

<!--

Thanks to: http://stackoverflow.com/questions/11256433/how-to-show-math-equations-in-general-githubs-markdownnot-githubs-blog

- http://latex.codecogs.com/gif.latex?d(x,y)=1/\sqrt{2}*\sqrt{\sum_{i=1}^k(\sqrt{x_i}-\sqrt{y_i})^2} 

-->

* [Container.hpp](https://github.com/mrquincle/particlefilter/blob/master/inc/Container.hpp) which contains distance functions (Euclidean, Battacharyya, Hellinger, Manhattan, Chebyshev) for standard C++ containers. For example the Hellinger distance is ![equation](http://latex.codecogs.com/gif.latex?d%28x%2Cy%29%3D1%2F%5Csqrt%7B2%7D*%5Csqrt%7B%5Csum_%7Bi%3D1%7D%5Ek%28%5Csqrt%7Bx_i%7D-%5Csqrt%7By_i%7D%29%5E2%7D).
* [Autoregression.hpp](https://github.com/mrquincle/particlefilter/blob/master/inc/Autoregression.hpp) with three nice utility template functions, one of them does calculate the actual autoregression, the others rotate or perform an automic "push-pop" operation. The latter is convenient if your data container does not happen to be a deque, but for example a vector.
* [Print.hpp](https://github.com/mrquincle/particlefilter/blob/master/inc/Print.hpp) in case you print comma-separated data containers content all the time.
* [File.hpp](https://github.com/mrquincle/particlefilter/blob/master/inc/File.hpp) get files from a directory without any dependencies (such as boost).

Except for the CImg template library, there have been two files used for demonstrating the particle filter:

* [ConfigFile.hpp](https://github.com/mrquincle/particlefilter/blob/master/inc/Container.hpp) which is written by Richard Wagner and makes it extremely easy to quickly write back and forth to configuration files.
* [alphanum.hpp](https://github.com/mrquincle/particlefilter/blob/master/inc/alphanum.hpp) written by Dirk Jagdmann.

## Where can I read more?
* [Wikipedia](http://en.wikipedia.org/wiki/Particle_Filter)

## Copyrights
The copyrights (2012) belong to:

- License: LGPL v.3
- Author: Anne van Rossum
- Almende B.V., http://www.almende.com and DO bots B.V., http://www.dobots.nl
- Rotterdam, The Netherlands

Note, ideas on using which algorithms (for example the specific autoregressive model) are inspired by [Rob Hess](http://blogs.oregonstate.edu/hess/code/particles/) for tracking a football player in a sequence of video frames. However, none of Rob's code has been used. This is an independent implementation.
