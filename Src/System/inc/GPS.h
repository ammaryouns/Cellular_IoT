//==============================================================================
//
//  GPS.h
//
//  Copyright (C) 2014 by Industrial Scientific.
//
//  This document and all  contained within are confidential and
//  proprietary property of Industrial Scientific Corporation. All rights 
//  reserved. It is not to be reproduced or reused without the prior approval 
//  of Industrial Scientific Corporation.
//
//==============================================================================
//  FILE 
//==============================================================================
//
//  Source:        GPS.h
//
//  Project:       Frey
//
//  Author:        Abdul Basit
//
//  Date:          2018/06/28
//
//  Revision:      1.0
//
//==============================================================================
//  FILE DESCRIPTION
//==============================================================================
//
//! \file
//! This file contains functions for GPS configuration and location based services   
//! 
//

#ifndef __GPS_H
#define __GPS_H
//==============================================================================
//  INCLUDES
//==============================================================================
#include <stdint.h>
#include <em_usart.h>
#include <uartdrv.h>

#include "CellularATCommands.h"
#include "ExtCommunication.h"
#include "Event.h"
//==============================================================================
//  GLOBAL CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================
#define FIND_MIN(X,Y)   ( (X) < (Y) ? (X) : (Y) )






#endif