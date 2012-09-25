/**
 * @brief 
 * @file imgbuffer.hpp
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


#ifndef IMGBUFFER_HPP_
#define IMGBUFFER_HPP_

// General files
#include <chunkbuffer.hpp>

#include <strstr.h>

using namespace std;

#define CHUNKSIZE 1460

/* **************************************************************************************
 * Interface of imgbuffer
 * **************************************************************************************/

#define CHUNK_BUFFER_SIZE  (1024*1024)

class imgbuffer: public chunkbuffer<char,(uint32_t)CHUNK_BUFFER_SIZE> {
public:
	//! Constructor imgbuffer
	imgbuffer(): frame_number(0), debug(5) {}

	//! Destructor ~imgbuffer
	virtual ~imgbuffer() {}

	/**
	 * Check item content for errors
	 */
	bool check_item_errors() {
		// the DCS-900 sends this spelling error in my firmware if it does not understand nof_bytes_read packet.
		if( sstrnstr(last_item_begin, "unknwon", current_item_size()) ) {
			if (debug)
				fprintf(stderr, "DCS-900 DETECTED UNKNOWN DATA, NETWORK PROBLEM?\n");
			return true;
		}
		return false;
	}

	/**
	 * Check from the start of the block and returns the size of the item. There are two interpretations of the
	 * size of the item. The "content size" is a lower bound on this and denotes for example the size of an image
	 * without the HTTP header. The "header size" is the header, so the total size of the thing in the stream
	 * including all kind of header info is "header_size" + "content_size".
	 */
	bool get_item_size(uint32_t & header_size, uint32_t & content_size) {
		char *ptr;
		int content_length;
		uint32_t size = current_item_size();

		//! search the string "image/jpeg" in the buffer
		ptr = sstrnstr(last_item_begin, "image", size);
		if(!ptr) {
			cerr << __func__ << ": could not find \"image/jpeg\" in chunk of size " << size << endl;
			if (size > 45000) exit(-1);
			return false;
		}
		if (debug)
			cout << __func__ << ": found Content-type: image/jpeg" << endl;

		// jump over string "image/jpeg"
		ptr = sstrnstr(last_item_begin, "Content-length: ", size);
		if(!ptr) {
			fprintf(stderr, "could not find \"Content-length: \", looping for more\n");
			return false;
		}
		if (debug)
			cout << __func__ << ": found Content-length" << endl;

		// read content length
		sscanf(ptr + 16, "%d", &content_length);
		cout << __func__ << ": content length = " << content_length << endl;

		header_size = 0;
		char *p;
		for (p = last_item_begin; p < last_item_begin+size; ++p) {
			if (((unsigned char)*p == 255) && ((unsigned char)*(p+1) == 216)) { //'0xFF' and '0xD8'
				header_size = p - last_item_begin;
				cout << __func__ << ": header size is " << header_size << endl;
				break;
			}
		}
		if (header_size == 0) return false;

		content_size = content_length;
		return true;
	}

	/**
	 * Read bytes from socket and put them in the ringbuffer
	 */
	int read_from_socket(int socketfd) {
		int nof_bytes_read = 0;
		nof_bytes_read = read(socketfd, cbuffer, CHUNKSIZE);
		if(nof_bytes_read > 0) {
			chunk<char> c;
			c.start = cbuffer;
			c.size = nof_bytes_read;
			addchunk(c);
			if (debug)
				fprintf(stderr, "read() returned %d bytes\n", nof_bytes_read);
			return nof_bytes_read;
		}
		fprintf(stderr, "mcamip: read(): returned EOF (power failure, network, interference?)\n");

		return nof_bytes_read;
	}

	/**
	 * Returns true if item is received and returns item size too (with respect to last_item_begin).
	 */
	bool item_received(int & item_size) {
		uint32_t content_size;
		uint32_t header_size;
		bool success;
		success = get_item_size(header_size, content_size);
		if (!success) return false;

		char *end_ptr = last_item_begin + content_size + header_size;
//		char *end_ptr = last_item_begin + header_size;

		int goback = 10;

		cout << __func__ << ": search from " << (end_ptr - goback) - last_item_begin \
				<< " to " << last_chunk_end - last_item_begin << endl;

		// check if end of picture is indeed 0xFF 0xD9
		//for ( char *p = end_ptr - goback; p < last_chunk_end; ++p) {
		for ( char *p = end_ptr - goback; p < end_ptr; ++p) {
			if (((unsigned char)*p == 255) && ((unsigned char)*(p+1) == 217)) { //'0xFF' and '0xD9'
				item_size = p - last_item_begin + 2;
				if (debug) {
					cout << __func__ << ": item received with size " << item_size << endl;
					cout << __func__ << ": expected size was " << header_size + content_size << endl;
				}
				return true;
			}
		}
		return false;
	}

	/**
	 * Write image buffer to file. The argument is relative to the beginning of the last item.
	 */
	void write_image(uint32_t header_size, uint32_t size, std::string filename) {
		FILE *pFile = fopen(filename.c_str(), "wb");
		fwrite(header_size + last_item_begin, 1, size, pFile);
		fclose(pFile);
	}

	inline void update_frame_number() {
		++frame_number;
	}

	inline int get_frame_number() { return frame_number; }


protected:

private:
	char cbuffer[CHUNKSIZE];

	int frame_number;

	char debug;
};

#endif /* IMGBUFFER_HPP_ */
