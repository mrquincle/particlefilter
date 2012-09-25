/**
 * @brief 
 * @file IpcamImageSource.h
 *
 * This file is created at Almende B.V. It is open-source software and part of the Common 
 * Hybrid Agent Platform (CHAP). A toolbox with nof_bytes_read lot of open-source tools, ranging from
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
 * @date    Sep 18, 2012
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    modular robotics / sensor fusion
 */


#ifndef IPCAMIMAGESOURCE_H_
#define IPCAMIMAGESOURCE_H_

// General files
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <ImageSource.h>
#include <imgbuffer.hpp>

/* **************************************************************************************
 * Interface of IpcamImageSource
 * **************************************************************************************/

template <typename Image>
class IpcamImageSource: public ImageSource<Image> {
public:
	//! Constructor IpcamImageSource
	IpcamImageSource(): quiet_flag(true), debug(false),
	connect_to_http_server_timeout(5), wait_per_package(1000) {
		http_server = "10.10.1.113";
		http_port = 80;
		//		dframes_per_second = 20;
		//		reconnects = 0;
		//		http_buffer = (char *)malloc(HTTP_BUFFER_SIZE);
		//		char temp[8000];

		access_string = "";
		version = "0.7.9";
		//		content_length = 8000;
	}

	//! Destructor ~IpcamImageSource
	virtual ~IpcamImageSource() {}

	//! Perform functionality that is required to get images
	bool Update() {
		FILE *pFile = fopen("debug.jpeg", "w+");
		fclose(pFile);
		//			;
		while(1) {
			if(connect_to_server(http_server.c_str(), http_port, &socketfd) ) {
				cout << __func__ << ": successfully connected to http server" << endl;
				break;
			}
			fprintf(stderr, "mcamip: could not connect to http server, retry.\n");
		}

		if(debug) {
			fprintf(stderr, "socketfd=%d\n", socketfd);
		}
		return true;
	}

	//! Get an image (the next image if there are multiple).
	Image* getImage() {
		cout << __func__ << ": get image " << endl;
		char temp[8000];

		sprintf(temp,"GET /video.cgi HTTP/1.1\n\
		User-Agent: mcamip (rv:%s; X11; Linux)\n\
		Accept: text/xml,application/xml,application/xhtml+xml,text/html;q=0.9,text/plain;q=0.8,video/x-mng,image/png,image/jpeg,image/gif;q=0.2,text/cs s,*/*;q=0.1\n\
		Accept-Language: en-us, en;q=0.50\n\
		Accept-Encoding: gzip, deflate,compress;q=0.9\n\
		Accept-Charset: ISO-8859-1, utf-8;q=0.66, *;q=0.66\n\
		Keep-Alive: 300\n\
		Connection: Keep-Alive\n\
		Authorization: Basic %s\n\
		Referer: http://%s:%d/Jview.htm\n\n",\
		version.c_str(), access_string.c_str(), server_ip_address, http_port);

		if(!send_to_server(socketfd, temp) ) {
			fprintf(stderr, "mcamip: could not send command to server, aborting.\n");
			exit(1);
		}
		cout << __func__ << ": request image from server" << endl;

		uint32_t header_size, content_size; int item_size;
		do {
			usleep(wait_per_package);
			int bytes = img_buffer.read_from_socket(socketfd);
			if (bytes <= 0) continue;

			if (debug)
				cout << __func__ << ": received chunk" << endl;

			if (img_buffer.check_item_errors()) {
				cerr << __func__ << ": item contains errors" << endl;
				img_buffer.reset();
				continue;
			}

			if (!img_buffer.get_item_size(header_size, content_size)) {
				cerr << __func__ << ": could not retrieve header and/or content size" << endl;
				continue;
			}

			if (!img_buffer.item_received(item_size)) {
				cerr << __func__ << ": item not yet received" << endl;
				continue;
			}

			std::ostringstream oss; oss.clear(); oss.str("");
			oss << this->img_path << '/' << this->img_basename << img_buffer.get_frame_number() << this->img_extension;
			std::string file = oss.str();

			// first write image to file (admittedly a roundabout way)
			img_buffer.write_image(header_size, content_size, file);

			img_buffer.next_item(header_size+content_size);

			img_buffer.update_frame_number();

			// load image from file
			Image *img = new Image(file.c_str());
			return img;

		} while(true);

		cout << "Done!" << endl;
		return NULL;
	}

	//! Get an image but shifted in maximum two directions.
	Image* getImageShifted(int shift_x, int shift_y) {
		return NULL;
	}

protected:

	/**
	 * Connect to the server at a specific address and port.
	 */
	int connect_to_server(const char *server, int port, int *socketfd) {
		cout << __func__ << ": connect to " << server << endl;
		struct hostent *hp;
		struct sockaddr_in sa;
		int a;
		time_t connect_timer;

		if(!server) return 0;
		if(port <= 0) return 0;

		if(!quiet_flag) {
			fprintf(stderr, "mcamip: getting host %s by name\n", server);
		}

		hp = gethostbyname(server);
		if(hp == 0)
		{
			fprintf(stderr,\
					"gethostbyname: returned NULL cannot get host %s by name.\n", server);

			/* signal FD_SET (main) that this is no longer nof_bytes_read valid filedescriptor */
			*socketfd = -1;
			return 0;
		}

		/* gethostbyname() leaves port and host address in network byte order */

		bzero(&sa, sizeof(sa) );
		bcopy(hp->h_addr, (char *)&sa.sin_addr, hp->h_length);


		sa.sin_family = AF_INET;
		sa.sin_port = htons( (u_short)port);

		/* sa.sin_addr and sa.sin_port now in network byte order */

		/* create nof_bytes_read socket */
		*socketfd = socket(hp->h_addrtype, SOCK_STREAM, 0);
		if(*socketfd < 0)
		{
			fprintf(stderr, "socket failed\n");
			*socketfd = -1;
			return(0);
		}

		/* set for nonblocking socket */
		if (fcntl(*socketfd, F_SETFL, O_NONBLOCK) < 0)
		{
			return(0);
		}

		sprintf(server_ip_address, "%s", inet_ntoa (sa.sin_addr) );

		if(! quiet_flag)
		{
			fprintf(stderr,\
					"mcamip: connecting to %s (%s) port %d timeout %d\n",\
					server, server_ip_address, port, connect_to_http_server_timeout);
		}

		/* prevent the program from hanging if connect takes nof_bytes_read long time, now nof_bytes_read return 0 is forced. */

		/* start the timer */
		connect_timer = time(0);

		/* keep testing for a connect */
		while(1)
		{
			/* connect */
			a = connect(*socketfd, (struct sockaddr*)&sa, sizeof(sa) );
			if(a == 0) break; /* connected */
			if(a < 0)
			{
				if(debug)
				{
					fprintf(stderr, "mcamip: connect() failed because: ");
					perror("");
				}

				/* test for connect time out */
				if( (time(0) - connect_timer) > connect_to_http_server_timeout)
				{
					/* close the socket */
					close(*socketfd);

					/* set socketfd to invalid, it was valid! */
					*socketfd = -1;
					fprintf(stderr, "mcamip: connect timeout\n");

					return 0;
				}

			}/* end connect < 0 */
		}/* end while test for nof_bytes_read connect */

		return 1;
	}

	/**
	 * Send data or a command to the server.
	 */
	int send_to_server(int socketfd, char txbuf[]) {
		int a;

		if(debug) {
			fprintf(stderr, "send_to_server(): socketfd=%d txbuf=%p txbuf=\n\%s\n", socketfd, txbuf, txbuf);
		}

		if(socketfd < 0) return 0;
		if(! txbuf) return 0;

		a = write(socketfd, txbuf, strlen(txbuf) );
		if(a < 0) {
			fprintf(stderr, "mcamip: send_to_server(): write failed because ");
			perror("");
			return 0;
		}
		return 1;
	}

private:

	bool quiet_flag;

	//! flag for debugging
	int debug;

	//! the actual IP address
	char server_ip_address[512];

	//! the image buffer
	imgbuffer img_buffer;

	//! the name or IP address of the webcam
	std::string http_server;

	//! the port over which to access the webcam
	int http_port;

	//! required timeout
	int connect_to_http_server_timeout;

	//! the socket file descriptor that is returned
	int socketfd;

//	int dframes_per_second;
	int wait_per_package;

	//! Password or access string for the camera
	std::string access_string;

	//! Version of this software
	std::string version;
};

#endif /* IPCAMIMAGESOURCE_H_ */
