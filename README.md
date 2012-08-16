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
Current phase: alpha. And not with respect to the [series](http://en.wikipedia.org/wiki/Alphas) but with respect to the fact that all functionality is there, but it needs to be debugged.

## What are the alternatives?
There are many alternatives for tracking, done here by particle filtering, ranging from Kalman filters to (other types of) Bayes filters. Then there are many forms of particle filters, for example the boosted particle filter, the unscented particle filter, the cascade particle filter, the kernel particle filter, the bootstrap particle filter and the mean-shift embedded particle filter. And last, but not least, there are software alternatives, such as the particle filter implementation in the [Mobile Robot Programming Toolkit](http://www.mrpt.org/Particle_Filters).

## An example
An example of the particle filter being able to track the docking element of a Replicator robot will follow soon.

## Where can I read more?
* [Wikipedia](http://en.wikipedia.org/wiki/Particle_Filter)

## Copyrights
The copyrights (2012) belong to:

- Author: Anne van Rossum
- Almende B.V., http://www.almende.com and DO bots B.V., http://www.dobots.nl
- Rotterdam, The Netherlands
