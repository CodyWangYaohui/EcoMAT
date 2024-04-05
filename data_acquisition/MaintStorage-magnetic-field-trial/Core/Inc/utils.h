/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : utils.h
 * @brief          : here contains all utility functions DEFs for retrieving sensor data, filtering sensor data, converting received strength data to distance, etc.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 Cody Wang.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 ******************************************************************************
 ******************************************************************************
 **/

// utils.h
#ifndef UTILS_H
#define UTILS_H

// Struct and function declarations...




// Define a point data structure
struct Point {
    double x;
    double y;
};

struct OverallPoints {
	struct Point Anchor_0;
	struct Point Anchor_1;
	struct Point Anchor_2;
	struct Point Anchor_3;
	struct Point Anchor_4;

};


/**
 * A static global object for RETAINING ALL SENSOR LOCATIONS
 */
static struct OverallPoints allAnchorLoc = {     {0.0,   0.0},      // anchor_0 position
																    {0.0, 50.5},      // anchor_1 position
																{38.0, 25.25},      // anchor_2 position
																	 {76, 50.5},	  // anchor_3 position
																	   {76, 0.0},	  // anchor_4 position
															   	   	   	   	   	   	   };  // current estimated location of the magnet in single magnet localization



#endif // UTILS_H
