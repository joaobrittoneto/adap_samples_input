#ifndef _SAMPLES_DATATYPE_H_
#define _SAMPLES_DATATYPE_H_

/******************************************************************************/
/*  Preparing data from Avalon to apply the adaptive parameters identification
/*
/*
/* PURPOSE ---
/*
/*  João Da Costa Britto Neto
/*  joao.neto@dfki.de
/*  DFKI - BREMEN 2014
/*****************************************************************************/

#include <iostream>
#include "base/samples/RigidBodyState.hpp"
#include "base/samples/RigidBodyAcceleration.hpp"
#include "base/samples/Joints.hpp"
#include "base/Eigen.hpp"
#include "base/Time.hpp"


namespace adap_samples_input
{
	struct DynamicAUV
		{
			base::samples::RigidBodyState rbs;
			base::samples::RigidBodyAcceleration rba;
			base::samples::Joints joints;
			base::Time time;
		};



} // end namespace adap_samples_input

#endif // _SAMPLES_DATATYPE_H_
