/**
 * @brief 
 * @file chunkbuffer.hpp
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
 * @date    Sep 21, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
 */


#ifndef CHUNKBUFFER_HPP_
#define CHUNKBUFFER_HPP_

// General files
#include <cassert>

template <typename T>
struct chunk {
	T *start;
	int size;
};

/* **************************************************************************************
 * Interface of chunkbuffer
 * **************************************************************************************/

/**
 * In many situations it seems you will need a ringbuffer. However, the situation is such
 * that you know more or less how big the items are you will need to retrieve from a
 * continuous stream of data. And you have plenty of space available, at least enough to
 * store say 20 of these items. The data comes continuous, but in chunks. In that case
 * it is convenient to use this chunkbuffer. You can add chunks of data, and items are
 * constructed out of these chunks.
 */
template <typename T, uint32_t size>
class chunkbuffer {
public:
	//! Constructor chunkbuffer
	chunkbuffer(): last_item_begin(buffer), last_chunk_end(buffer) {
		 }

	//! Destructor ~chunkbuffer
	virtual ~chunkbuffer() {
	}

	/**
	 * Add a chunk to the buffer. If there is not enough space, move all chunks that
	 * belong to the last item to the beginning.
	 */
	void addchunk(chunk<T> c) {
		if (c.size >= remain_to_end()) {
			move_to_begin();
		}
		memcpy(last_chunk_end, c.start, c.size);
		last_chunk_end += c.size;
	}

	/**
	 * The start of the next item is not the same as the "last_chunk_end". It can be
	 * halfway the last chunk. This is not set in "item_received" because that would
	 * screw up the "last_item_begin" of the item that has yet to be received.
	 */
	void next_item(uint32_t skip) {
		assert (last_item_begin + skip <= last_chunk_end);
		last_item_begin += skip;
	}

	//! Check item for errors
	virtual bool check_item_errors() = 0;

	//! Get the size of the item (should be total of header_size + content_size)
	virtual bool get_item_size(uint32_t & header_size, uint32_t & content_size) = 0;

	//! Returns true if item has been received (will probably call get_item_size)
	virtual bool item_received(int & item_size) = 0;

	/**
	 * Physically copy the chunks belonging to the last item to the beginning of the
	 * buffer. The rest of the buffer will be overwritten.
	 */
	void move_to_begin() {
		uint32_t already_there = last_chunk_end - last_item_begin;
		cout << __func__ << ": move all last chunks to beginning (size=" << already_there << ")" << endl;
		memcpy(buffer, last_item_begin, already_there);
		last_item_begin = buffer;
		last_chunk_end = buffer+already_there;
	}

	/**
	 * Remaining size of the buffer.
	 */
	inline uint32_t remain_to_end() {
		uint32_t r = size - (last_chunk_end - buffer);
		cout << __func__ << ": space = " << (int)r << " (should be < " << size << ")" << endl;
		return r;
	}

	inline void reset() {
		last_item_begin = last_chunk_end = buffer;
	}

	/**
	 * Current item size.
	 */
	inline uint32_t current_item_size() {
		uint32_t s = last_chunk_end - last_item_begin;
		cout << __func__ << ": size = " << (int)s << endl;
		return s;
	}

protected:

	T buffer[size];

	T* last_item_begin;

	T* last_chunk_end;

private:
};

#endif /* CHUNKBUFFER_HPP_ */
