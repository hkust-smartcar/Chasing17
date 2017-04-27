#include <vector>
#include <utility>
#include <cstdint>

#include "car_manager.h"

#ifndef CHASING17_ALGORITHM_OPTIMAL_MAIN_H_
#define CHASING17_ALGORITHM_OPTIMAL_MAIN_H_

namespace algorithm{
namespace optimal{
void main(CarManager::ServoBounds);

typedef struct {
	std::vector<std::pair<int, int>> points;
	inline void push(int x, int y) {points.push_back(std::make_pair(x, y));}
	inline int size() {return points.size();}
	inline void insert(int pos, int x, int y) {points.emplace(points.begin() + pos, std::make_pair(x,y));}
} Edges;

struct {
	uint8_t w = 80, h = 60;
} CameraSize;

}
}
#endif //CHASING17_ALGORITHM_OPTIMAL_MAIN_H_
