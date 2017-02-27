/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 */

#pragma once

/**
 * 2017 Smartcar Internal - Center Line Method
 *
 * Computes the center line from the image, then uses the values to commit the
 * values to motor and servo.
 *
 * @note Used in the smartcar configuration 2016_INNO during the internal
 * competition.
 * @note All smartcar components are initialized in this method.
 */
void CenterLineMethod();

/**
 * 2017 Smartcar Phase B - Center Line Method (Test)
 *
 * Computes the center line from the image, then uses the values to commit the
 * values to servo and motor (via encoder adjustment).
 *
 * @note All smartcar components are initialized in this method.
 */
void CenterLineMethodTest();
