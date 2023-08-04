#include "controller.h"
#include "model/model.h"
#include "expander.h"
#include "slave.h"

void controller_init(model_t *model) {
    (void) model;

    expander_init();
    // expander_test();
    slave_init();
}

void controller_manage() {
    slave_manage();
}
