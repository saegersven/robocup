#include "utils.hpp"

float clip(float n, float lower, float upper) {
	return std::max(lower, std::min(n, upper));
}