/*
 * @file Statistics.h
 * @date Feb 21, 2012
 * @author Can Erdogan
 * @brief The generic statistics class that requires the templated object to have a function
 * that returns a hash value.
 */

#ifndef _STATISTICS_H_
#define _STATISTICS_H_

#include <stdlib.h>

#include <map>
#include <set>
#include <vector>

/// Define the statistics - keeps track of the n highest probable states throughout sampling
template <class OBJECT>
class Statistics {
public:

	/// The entry in the class that represents the object and the number of times it is updated
	struct Entry {
		const OBJECT* object_;
		size_t count_;
		Entry (const OBJECT* object, size_t count) : object_(object), count_(count) {}
	};

	std::map <long, Entry> entries_;						/// A map from the object hashes to the entries

	/// Updates the entries with the new object instance
	static void update (Statistics <OBJECT>& stats, const OBJECT& object) {

		// Check if this state is already in the statistics. If so, increment its count.
		bool found = false;
		long hash = object.hash_;
		typename std::map <long, Entry>::iterator entryIt = stats.entries_.find(hash);
		if(entryIt != stats.entries_.end()) {
			entryIt->second.count_++;
			return;
		}

		// Create an entry for the object
		Entry newEntry = Entry(&object, 1);
		stats.entries_.insert(std::make_pair(hash, newEntry));
	}

	/// Returns the objects with the most frequent one on the top.
	static void getOrderedStates (const Statistics<OBJECT>& stats, std::vector <Entry>& entries,
	    int topN = -1) {

		// Define a set of entries
		bool(*fn_pt) (Entry, Entry) = fncomp;
		std::set <Entry, bool(*) (Entry, Entry)> entriesSet(fn_pt);

		// Build the set of entries to sort them in terms of the counts
		typename std::map <long, Entry>::const_iterator cit = stats.entries_.begin();
		for (; cit != stats.entries_.end(); cit++)
			entriesSet.insert(cit->second);

		// Determine the number of entries to output
		size_t numEntries = entriesSet.size();
		if (topN != -1) numEntries = topN;

		// Output the requested number of entries
		typename std::set <Entry>::const_iterator setIt = entriesSet.begin();
		for (size_t i = 0; (setIt != entriesSet.end()) && (i < numEntries); setIt++, i++)
			entries.push_back(*setIt);
	}

	/// Returns the number of times an object is seen
	static size_t getCount (const Statistics<OBJECT>& stats, const OBJECT& object) {

		// Check if this state is already in the statistics. If so, increment its count.
		long hash = object.hash_;
		typename std::map <long, Entry>::const_iterator entryIt = stats.entries_.find(hash);
		if(entryIt != stats.entries_.end()) {
			return entryIt->second.count_;
		}

		return 0;
	}

private:

	/// The comparison function for the set of entries in getOrderedStates.
	/// NOTE The comparison is inverted to have the most probable states at the beginning of the
	/// returned vector.
	static bool fncomp (Statistics <OBJECT>::Entry e1, Statistics <OBJECT>::Entry e2) {
		if (e1.count_ < e2.count_) return true;
		else if (e1.count_ == e2.count_) return e1.object_ < e2.object_;
		return false;
	}
};

#endif // _STATISTICS_H_
