/*
 * LeNet.h
 *
 *  Created on: 2020. 2. 27.
 *      Author: floyed
 */
#ifndef __LENET_H__
#define __LENET_H__

//parameters
#include "parameters.h"

//layers
#include "activation.h"
#include "convolution.h"
#include "fullyconnected.h"
#include "pooling.h"

void LeNet(float src[BUFFER_SIZE], float dst[CLASSES], int id);

#endif
