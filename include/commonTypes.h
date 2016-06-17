/*
 * commonTypes.h
 *
 *  Created on: 15.04.2016
 *      Author: Marcin.Karczewski
 */

#ifndef INCLUDE_COMMONTYPES_H_
#define INCLUDE_COMMONTYPES_H_

enum ERRORTYPE
{
	ET_OK 				= 0,
	ET_NOK 				= 1,
	ET_PENDING  		= 2,
	ET_ERROR			= 3,
	ET_ARG_TOO_BIG 		= 4,
	ET_I2C_BUSY 		= 5,
	ET_NULL_ARG			= 6,
	ET_NOT_INITIALIZED  = 7
};

#endif /* INCLUDE_COMMONTYPES_H_ */
