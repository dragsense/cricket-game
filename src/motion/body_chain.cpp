#include "motion/body_chain.h"

#include <algorithm>
#include <cmath>

namespace motion {

float chainOwnPart(float need, float limit, float onset) {
    if (limit <= 0.0f) return 0.0f;
    const float start = std::clamp(onset, 0.0f, 1.0f) * limit;
    if (need <= start) return need;
    const float room = limit - start;  // what is left of the joint's range past the onset
    if (room <= 0.0f) return limit;
    // Exponential approach: slope 1 at the onset, the limit as need -> inf.
    return start + room * (1.0f - std::exp(-(need - start) / room));
}

} // namespace motion
