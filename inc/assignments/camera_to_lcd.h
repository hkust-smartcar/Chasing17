/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 */

#pragma once

/**
 * Retrieves an image from the camera and displays it on the LCD. This process
 * will stop when the retrieve-copy sequence takes longer than @c test_ms
 * milliseconds.
 *
 * @note Camera and LCD will be initialized in the function.
 */
void CameraToLcd();
