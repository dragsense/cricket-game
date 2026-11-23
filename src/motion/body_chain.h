#pragma once
// The body's chain rule (the user's design, 2026-11-23): a joint takes what is
// asked of it on its own, and as it comes near its limit the next one in the
// chain (toward the hips from a hand or foot, on from the hips) takes over,
// smoothly, never with a jump at the limit. One rule for every link (hip to
// pelvis, arm to shoulder to chest to pelvis, ...). Pure maths, no state.

namespace motion {

// How much of `need` a joint with room `limit` (same units, > 0) takes
// itself; the rest (need - result) is the next joint's. It takes all of it up
// to onset * limit; beyond, its part bends over smoothly (same slope at the
// onset) toward the limit, which it reaches only as the need grows without
// end. `need` at or below the onset (or a negative one) comes back unchanged.
float chainOwnPart(float need, float limit, float onset);

} // namespace motion
