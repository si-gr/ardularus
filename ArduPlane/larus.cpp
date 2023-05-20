#include "Plane.h"

void Plane::update_larus() {
    plane.larus_controller.update(plane.g2.soaring_controller.get_thermability(), plane.g2.soaring_controller.get_vario_reading(), plane.g2.soaring_controller.get_thermalling_radius());
}