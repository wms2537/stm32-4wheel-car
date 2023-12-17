/*
 * types.h
 *
 *  Created on: Dec 16, 2023
 *      Author: sohweimeng
 */

#ifndef SRC_TYPES_HPP_
#define SRC_TYPES_HPP_

struct Position_edc25
{
    float posx;
    float posy;
    int coordx() {
    	return (int)posx;
    }
    int coordy() {
		return (int)posy;
	}
    int chunkid() {
    	return coordx()+coordy()*8;
    }
} ;


#endif /* SRC_TYPES_HPP_ */
