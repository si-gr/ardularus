#include "Plane.h"

void Plane::update_larus() {
    plane.g2.soaring_controller.update_thermalling();
    plane.larus_controller.update(plane.g2.soaring_controller.get_thermability(), plane.g2.soaring_controller.get_vario_reading(), plane.g2.soaring_controller.get_thermalling_radius(), plane.g2.soaring_controller.get_e0(), plane.g2.soaring_controller.get_e1(), plane.g2.soaring_controller.get_e2(), plane.g2.soaring_controller.get_e3());
}