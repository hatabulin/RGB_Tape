/*
 * Utils.cpp
 *
 *  Created on: 8 îêò. 2019 ã.
 *      Author: serg
 */

#include "Utils.h"

void strcopy(char* main, char* second, int pos, int k) {
    if (pos > strlen(main)) {
        second[0] = 0;
        return;
    }
    int i = 0;
    for (; i < k; ++i) second[i] = main[pos++];
    second[i] = 0;
}
