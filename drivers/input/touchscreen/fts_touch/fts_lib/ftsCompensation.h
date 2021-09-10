/*
  *
  **************************************************************************
  **                        STMicroelectronics				  **
  **************************************************************************
  **                        marco.cali@st.com				**
  **************************************************************************
  *                                                                        *
  *               FTS functions for getting Initialization Data		  **
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsCompensation.h
  * \brief Contains all the definitions and structs to work with Initialization
  * Data
  */

#ifndef FTS_COMPENSATION_H
#define FTS_COMPENSATION_H

#include "ftsCore.h"
#include "ftsSoftware.h"



#define RETRY_FW_HDM_DOWNLOAD 2	/* /< max number of attempts to
				 * request HDM download */


/* Bytes dimension of HDM content */

#define HDM_DATA_HEADER	DATA_HEADER	/* /< size in bytes of
						 * initialization data header */
#define COMP_DATA_GLOBAL	(16 - HDM_DATA_HEADER)	/* /< size in bytes
							 * of initialization
							 * data general info */
#define GM_DATA_HEADER		(16 - HDM_DATA_HEADER)	/* /< size in bytes of
 						* Golden Mutual data header */


#define HEADER_SIGNATURE	0xA5	/* /< signature used as starting byte of
					 * data loaded in memory */



/**
  * Struct which contains the general info about Frames and Initialization Data
  */
typedef struct {
	int force_node;	/* /< Number of Force Channels in the
			 * frame/Initialization data */
	int sense_node;	/* /< Number of Sense Channels in the
			 * frame/Initialization data */
	int type;	/* /< Type of frame/Initialization data */
} DataHeader;

#endif
