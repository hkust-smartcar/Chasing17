/*
 * main.h
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Peter Tse (mcreng)
 *
 * Optimal Path Algorithm Header File
 *
 */

#include <vector>
#include <list>
#include <utility>
#include <cstdint>

#include "car_manager.h"

#ifndef CHASING17_ALGORITHM_OPTIMAL_MAIN_H_
#define CHASING17_ALGORITHM_OPTIMAL_MAIN_H_

namespace algorithm{
namespace optimal{
void main(CarManager::ServoBounds);

/**
 * Edges struct
 *
 * An type implementation for storage of Edges
 * @member points Vector storing the edges sequentially
 * @member push(int, int) Push a std::pair<int, int> into the vector points
 * @member push(Edges) Push a Edges into the vector points
 * @member size() Return the size of vector points
 * @member insert(int, int, int) Insert a std::pair<int, int> into some position of vector points
 * @member insert(int, Edges) Insert an Edges type into some position of vector points
 * @member grad() Take the gradient of certain Edges
 */
struct Edges {
	std::vector<std::pair<int, int>> points;
	inline void push(int x, int y) {points.push_back(std::make_pair(x, y));}
	inline void push(Edges edge) {points.insert(points.end(), edge.points.begin(), edge.points.end());}
	inline int size() {return points.size();}
	inline void insert(int pos, int x, int y) {points.emplace(points.begin() + pos, std::make_pair(x,y));}
	inline void insert(int pos, Edges edge) {points.insert(points.begin() + pos, edge.points.begin(), edge.points.end());}
	Edges grad(){
		Edges temp;
		for (int i = 1; i < this->size(); i++){
			auto last = this->points[i];
			auto second_last = this->points[i-1];
			temp.push(last.first - second_last.first, last.second - second_last.second);
		}
		return temp;
	}
};

/**
 * Corners struct
 *
 * An type implementation for storage of Corners
 * @member points List storing the edges sequentially
 * @member push(int, int) Push a std::pair<int, int> into the list points
 * @member size() Return the size of list points
 */
struct Corners {
	std::list<std::pair<int, int>> points;
	inline void push(int x, int y) {points.push_back(std::make_pair(x, y));}
	inline int size() {return points.size();}
};

struct {
	uint16_t w = 128, h = 480;
} CameraSize;

struct {
	uint16_t w = 128, h = 160;
} WorldSize;

/**
 * TranslateType enum struct
 *
 * An enum struct implementation designed for GenPath()
 */
enum struct TranslateType {
	kNone = 0, kLeftNull, kRightNull
};

}
}
#endif //CHASING17_ALGORITHM_OPTIMAL_MAIN_H_
