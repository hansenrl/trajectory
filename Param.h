#ifndef __PARAM_H__
#define __PARAM_H__

#define __COMBO_II__
#if defined(__COMBO_I__)
#undef __USE_CONVENTIONAL_PARTITONING__
#define __USE_NO_PARTITIONING__
#undef __PARTITION_PRUNING_OPTIMIZATION__
#else if defined(__COMBO_II__)
#define __USE_CONVENTIONAL_PARTITONING__
#undef __USE_NO_PARTITIONING__
#define __PARTITION_PRUNING_OPTIMIZATION__
#endif
#undef __SHOW_TRAJECTORY_PARTITION__
#define __INCORPORATE_DENSITY__
#define __PRECOMPUTE_DENSITY__
#undef __VISUALIZE_DEBUG_INFO__

// This header file contains all the tuning parameters for the outlier detection algorithm

const float g_FRACTION_PARAMETER = (float)0.95;
const float g_DISTANCE_PARAMETER = (float)82.0; // (float)80.0;
const float g_MINIMUM_OUTLYING_PROPORTION = (float)0.20; // (float)0.10;

const int MDL_COST_ADVANTAGE = 20;
const float MIN_LINESEGMENT_LENGTH = 1.0;
const float MAX_LINESEGMENT_LENGTH = 10000.0;	// 100.0 only for deer

#define WEIGHTED_DISTANCE(_x,_y,_z) ((float)1.0 * (_x) + (float)1.0 * (_y) + (float)10.0 * (_z))
// #define WEIGHTED_DISTANCE(_x,_y,_z) ((float)1.0 * (_x) + (float)1.0 * (_y) + (float)5.0 * (_z))

#define RESULT_FILE "C:\\experiments\\trajectory outlier\\result\\result.txt"

#endif