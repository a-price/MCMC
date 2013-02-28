/*
 * @file MCMC.h
 * @date Sept 21, 2011
 * @author Frank Dellaert
 * @author Can Erdogan
 * @author Manohar Paluri
 * @brief Develop MCMC sampling algorithms
 */

#include <math.h>
#include <stdio.h>
#include <algorithm>

#include <gtsam/base/timing.h>

#include "Statistics.h"

/// Base class for all MCMC samplers
template <class PROBLEM>
class MCMC {
public:

	/// The statistics that is gathered.
	Statistics <typename PROBLEM::State> statistics;

	/**
	 * Over-write this method for different MCMC variants
	 * @param state PROBLEM::State which describes the state within the problem context.
	 * @return returns the new state after one iteration of the sampler.
	 */
	virtual typename PROBLEM::State* iterate (typename PROBLEM::State* state) const = 0;

	/**
	 * Generic MCMC sampler with burn-in
	 * @param nrBurnInIterations the number of iteration to run the sampler so that there is no bias
	 * towards the initial state
	 * @param nrIterations the number of iterations the sample is run to create a distribution of
	 * states.
	 * @param stateData The data that one might want to pass to the PROBLEM type for initializing the
	 * first state.
	 * @return Returns the statistics defined by the input problem.
	 */
	Statistics <typename PROBLEM::State> run (size_t nrBurnInIterations, size_t nrIterations,
	    void* data = NULL) {

		bool inform = true, informIter = false;

		// Initialize state
		typename PROBLEM::State* state = PROBLEM::initializeState(data);

		// Do a burn-in period of nrBurnInIterations
		if(inform) fprintf(stderr, "--------------------------------------------------\n");
		if(inform) fprintf(stderr, "Starting burn-in iterations...\n");
		size_t step = std::max(1, (int) (nrBurnInIterations / 1000));
		for (size_t iteration = 0; iteration < nrBurnInIterations; iteration++) {
			if(informIter && iteration % step == 0) {
				fprintf(stderr, "Completed %ld iterations...\n", iteration);
				fflush(stdout);
			}
			state = iterate(state);
			gtsam::tictoc_finishedIteration_();
		}

		if(inform) fprintf(stderr, "Burn-in completed.\n");
		if(inform) fprintf(stderr, "--------------------------------------------------\n");
		if(inform) fprintf(stderr, "Starting main iterations...\n");

		// Collect statistics after burn-in
		step = std::max(1, (int) (nrIterations / 100));
		for (size_t iteration = 0; iteration < nrIterations; iteration++) {
			if(inform && (iteration % step == 0)) {
				printf("Completed %ld iterations...\n", iteration);
				fflush(stdout);
			}

			state = iterate(state);
			Statistics <typename PROBLEM::State>::update(statistics, (*state));
			gtsam::tictoc_finishedIteration_();
			// printf("count: %lu\n", Statistics <typename PROBLEM::State>::getCount(statistics, *state));
//			usleep(500000);

		}

		if(inform) printf("Main iterations completed.\n");
		if(inform) fprintf(stderr, "--------------------------------------------------\n");

		// return statistics to caller
		return statistics;
	}

	/// Returns the statistics
	Statistics <typename PROBLEM::State>& getStatistics () {
		return statistics;
	}
};

/**
 * Metropolis sampler (symmetric proposal)
 */
template <class PROBLEM>
class Metropolis: public MCMC <PROBLEM> {

	/**
	 * One iteration of Metropolis does the following:
	 * 1) propose a new state
	 * 2) calculates acceptance probability
	 * 3) accepts probabilistically
	 * @param state PROBLEM::State which describes the state within the problem context
	 * @return returns the new state after one iteration of the sampler.
	 */
	virtual typename PROBLEM::State* iterate (typename PROBLEM::State* state) const {

		bool inform = true;

		// propose a new state
		if(inform) {
			printf("Proposing..\n");
			fflush(stdout);
		}
		typename PROBLEM::State* newState = PROBLEM::propose(*state);
//		if(logFile) newState->print("Proposed state: ", false, logFile);

// calculates acceptance probability
		if(inform) {
			printf("computing Acceptance ratio..\n");
			fflush(stdout);
		}
		long double newTarget = PROBLEM::target(*newState);
		long double currentTarget = PROBLEM::target(*state);
		long double a = newTarget / currentTarget;

		// accepts probabilistically
		bool accept = true;
		if(a < 1.0) {
			double r = ((double) rand()) / RAND_MAX;
			if(r > a) accept = false;
		}

		// Delete the old state if accepted
		if(accept) delete state;

		// Return new state (if accepted) or old state (if proposal rejected)
		typename PROBLEM::State* accepted = accept ? newState : state;
		return accepted;
	}

};

/**
 * Metropolis-Hastings sampler
 */
template <class PROBLEM>
class MetropolisHastings: public MCMC <PROBLEM> {
	/**

	 * One iteration of Metropolis-Hastings does the following:
	 * 1) propose a new state,
	 * 2) calculates acceptance probability,
	 * 3) accepts probabilistically
	 * @param state PROBLEM::State which describes the state within the problem context
	 * @return returns the new state after one iteration of the sampler.
	 */
	virtual typename PROBLEM::State* iterate (typename PROBLEM::State* state) const {

		// propose a new state
		typename PROBLEM::State* newState = PROBLEM::propose(*state);

		// calculates acceptance probability
		long double targetRatio = PROBLEM::target(*newState) / PROBLEM::target(*state);
		long double proposalRatioNumerator = PROBLEM::proposalDensity(*state, *newState);
		long double proposalRatioDenominator = PROBLEM::proposalDensity(*newState, *state);

		// If the next state is not possible then return the present state
		if(proposalRatioDenominator == 0) return state;

		// Compute the proposal ratio
		long double proposalRatio = proposalRatioNumerator / proposalRatioDenominator;

		// Compute the acceptance value
		long double a = targetRatio * proposalRatio;

		// accepts probabilistically
		bool accept = true;
		if(a < 1.0) {
			long double r = ((long double) rand()) / RAND_MAX;
			if(r > a) accept = false;
		}

		// Return new state (if accepted) or old state (if proposal rejected)
		return accept ? newState : state;
	}
};

/**
 * Fast Metropolis-Hastings sampler that let's the domain problem figure out the target and
 * proposal ratios using as much as domain knowledge possible
 */
template <class PROBLEM>
class FastMetropolisHastings: public MCMC <PROBLEM> {

	/**
	 * One iteration of Metropolis-Hastings does the following: 1) propose a new state, 2) calculates
	 * acceptance probability, and (3) accepts probabilistically.
	 * NOTE Assumes that the retrieved target ratio is in the log base, i.e. it is
	 * the log of the real ratios.
	 */
	typename PROBLEM::State* iterate (typename PROBLEM::State* state) const {

		static const bool debug = false;

		// Propose a new state and retrieve target and proposal ratios
		long double targetRatio, proposalRatio;
		gttic_(Proposals);
		typename PROBLEM::State* newState = PROBLEM::propose(*state, targetRatio, proposalRatio);
		gttoc_(Proposals);

		// Compute the acceptance probability
		long double aL = targetRatio + log(proposalRatio);
		long double a = exp(aL);

		// Accept probabilistically.
		// NOTE The comparison with aL has to be with 0.0 since log(a) >= 0.0 if a >= 1.0.
		long double r = ((long double) rand()) / RAND_MAX;
		bool accept = true;
		if(aL < 0.0) {
			if(r > a) accept = false;
		}

		if(debug) {
			printf("r: %Lf, aL: %Le (%Le), targetL: %Lf, proposalL: %Lf\n", r, aL, a, targetRatio,
			    proposalRatio);
			printf("\t%s\n", accept ? "Accepted." : "Rejected.");
			fflush(stdout);
		}

		// Return new state (if accepted) or old state (if proposal rejected)
		return accept ? newState : state;
	}
};
