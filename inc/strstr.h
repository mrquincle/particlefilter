/**
 * @brief 
 * @file strstr.h
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


#ifndef STRSTR_H_
#define STRSTR_H_

#include <unistd.h>

/*

int *kmp_borders(const char *needle, size_t nlen){
	if (!needle) return NULL;
	int i, j, *borders = (int*)malloc((nlen+1)*sizeof(*borders));
	if (!borders) return NULL;
	i = 0;
	j = -1;
	borders[i] = j;
	while((size_t)i < nlen){
		while(j >= 0 && needle[i] != needle[j]){
			j = borders[j];
		}
		++i;
		++j;
		borders[i] = j;
	}
	return borders;
}

char *kmp_search(char *haystack, size_t haylen, const char *needle, size_t nlen, int *borders){
	size_t max_index = haylen-nlen, i = 0, j = 0;
	while(i <= max_index){
		while(j < nlen && *haystack && needle[j] == *haystack){
			++j;
			++haystack;
		}
		if (j == nlen){
			return haystack-nlen;
		}
		if (!(*haystack)){
			return NULL;
		}
		if (j == 0){
			++haystack;
			++i;
		} else {
			do{
				i += j - (size_t)borders[j];
				j = borders[j];
			}while(j > 0 && needle[j] != *haystack);
		}
	}
	return NULL;
}

char *sstrnstr(char *haystack, const char *needle, size_t haylen){
	if (!haystack || !needle){
		return NULL;
	}
	size_t nlen = strlen(needle);
	if (haylen < nlen){
		return NULL;
	}
	int *borders = kmp_borders(needle, nlen);
	if (!borders){
		return NULL;
	}
	char *match = kmp_search(haystack, haylen, needle, nlen, borders);
	free(borders);
	return match;
}
*/

char *sstrnstr(char *haystack, const char *needle, size_t length)
{
    size_t needle_length = strlen(needle);
    size_t i;
    for(i = 0; i < length; i++)
    {
        if(i + needle_length > length)
        {
            return NULL;
        }
        if(strncmp(&haystack[i], needle, needle_length) == 0)
        {
            return &haystack[i];
        }
    }
    return NULL;
}



#endif /* STRSTR_H_ */
