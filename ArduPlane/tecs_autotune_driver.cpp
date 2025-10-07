#include "Plane.h"
#include <AP_TECS/AP_TECS_Autotune.h>

class PlaneTECSIO : public AP_TECS_AT_IO {
public:
    PlaneTECSIO(Plane& p): plane(p) {}
    float get_alt()       const override { return plane.relative_altitude(); }
    float get_alt_ref()   const override { return plane.auto_state.target_altitude; }
    float get_arsp()      const override { return plane.airspeed_estimate(); } // sensör yoksa estimate
    float get_arsp_ref()  const override { return plane.auto_state.target_airspeed; }
    float get_thr_out()   const override { return plane.throttle_output; }
    float get_pitch_cmd() const override { return degrees(plane.attitude_control->get_pitch_target_rad()); }
    void set_alt_ref(float h) override   { plane.set_target_altitude(h); }
    void set_arsp_ref(float v) override  { plane.set_target_airspeed(v); }
private:
    Plane& plane;
};

static AP_TECS_AT_Config _cfg;          // sahada paramını değiştirebilirsin
static PlaneTECSIO*      _io = nullptr;
static AP_TECS_Autotune* _at = nullptr;

void Plane::tecs_at_init()
{
    if (_io == nullptr)  _io = new PlaneTECSIO(*this);
    if (_at == nullptr)  _at = new AP_TECS_Autotune(*_io, _cfg);
}

void Plane::tecs_at_update()
{
    if (!_at) return;
    // Tetik: RC7 > 1800 ise tek atışlık bir ALT step başlat
    if (!_at->running() && (rc().read(7) > 1800)) {
        _cfg.step_dh_m = 50.0f;           // STE testi: +50 m
        _cfg.settle_band = 0.02f;         // ±%2
        _cfg.window_pre_s = 2.0f;
        _cfg.window_post_s = 20.0f;
        _cfg.target_settle_s = 8.0f;
        _cfg.target_overshoot_pct = 10.0f;
        _at->begin(AP_TECS_AT_Step::STE_ALT_STEP);
    }
    _at->update(); // ~50–100 ms sıklıkla çağırılacak
}
