#include "helpers.h"

void generate_KD_Tree(GLViewKD_Trees* glview, WO* wo, float plane1, float plane2, std::map<WO*, KD_Node*>& PlaneMap, int iteration) {
	if (iteration == 0) {
		return;
	}

	if (iteration % 3 == 0) { // x plane

	}
	else if (iteration % 3 == 1) { // y plane

	}
	else { // z plane

	}
}