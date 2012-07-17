/**
  @file src/omxcore.h

  OpenMax Integration Layer Core. This library implements the OpenMAX core
  responsible for environment setup, components tunneling and communication.

  Copyright (C) 2007  STMicroelectronics and Nokia

  This library is free software; you can redistribute it and/or modify it under
  the terms of the GNU Lesser General Public License as published by the Free
  Software Foundation; either version 2.1 of the License, or (at your option)
  any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
  details.

  You should have received a copy of the GNU Lesser General Public License
  along with this library; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St, Fifth Floor, Boston, MA
  02110-1301  USA

  $Date: 2008/08/21 09:26:02 $
  Revision $Rev: 239 $
  Author $Author: B060452 $
*/

#ifndef __ST_OMXCORE_H__
#define __ST_OMXCORE_H__

#include <OMX_Component.h>
#include <OMX_Types.h>

#include <pthread.h>
//#include <sys/msg.h>

#include "component_loader.h"
#include "omx_comp_debug_levels.h"

/** Defines the major version of the core */
#define SPECVERSIONMAJOR  1
/** Defines the minor version of the core */
#define SPECVERSIONMINOR  1
/** Defines the revision of the core */
#define SPECREVISION      0
/** Defines the step version of the core */
#define SPECSTEP          0

#endif
