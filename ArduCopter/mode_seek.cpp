// ArduCopter/ModeSeek.cpp
// #pragma once
#include "Copter.h"


#if MODE_SEEK_ENABLED

#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <limits>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
// -------- Optional GCS/MAVLink RSSI access ----------
#if __has_include("GCS_Mavlink.h")
#include "GCS_Mavlink.h"
#define HAVE_GCS_MAVLINK 1
#elif __has_include(<GCS_Mavlink/GCS_Mavlink.h>)
#include <GCS_Mavlink/GCS_Mavlink.h>
#define HAVE_GCS_MAVLINK 1
#else
#define HAVE_GCS_MAVLINK 0
#endif


const AP_Param::GroupInfo ModeSeek::var_info[] = {
    // Filtering
    AP_GROUPINFO("_W",           0, ModeSeek, P_W,            5),       // int
    AP_GROUPINFO("_ALPHA",       1, ModeSeek, P_ALPHA,        0.25f),
    AP_GROUPINFO("_FS",          2, ModeSeek, P_FS,           1.0f),


    // Decision / step shaping
    AP_GROUPINFO("_TURN_DEG",    3, ModeSeek, P_TURN_DEG,     90.0f),
    AP_GROUPINFO("_STEP_K",      4, ModeSeek, P_STEP_K,       10.0f),
    AP_GROUPINFO("_DECAY",       5, ModeSeek, P_DECAY,        5.0f),
    AP_GROUPINFO("_STEP_MIN",    6, ModeSeek, P_STEP_MIN,     0.25f),
    AP_GROUPINFO("_STEP_MAX",    7, ModeSeek, P_STEP_MAX,     20.0f),
    AP_GROUPINFO("_SPEED_MPS",   8, ModeSeek, P_SPEED_MPS,    0.5f),

    // Stop condition
    AP_GROUPINFO("_TARGET_DBM",  9, ModeSeek, P_TARGET_DBM,  -75.0f),
    AP_GROUPINFO("_FLOOR_DBM",  10, ModeSeek, P_FLOOR_DBM,  -100.0f),

    // Navigation tunables
    AP_GROUPINFO("_CRUISE_SPD", 11, ModeSeek, P_CRUISE_SPD,   1.2f),
    AP_GROUPINFO("_STOP_RAD",   12, ModeSeek, P_STOP_RAD,     2.0f),

    AP_GROUPEND
};



// TinyRNG methods
float ModeSeek::TinyRNG::uniform01()
{
    state = state * 1664525u + 1013904223u;
    return (float)((state >> 8) & 0x00FFFFFFu) / 16777216.0f;
}

float ModeSeek::TinyRNG::normal01()
{
    float u1 = clampf(uniform01(), 1e-6f, 1.0f - 1e-6f);
    float u2 = uniform01();
    const float mag = sqrtf(-2.0f * logf(u1));
    const float ang = 2.0f * float(M_PI) * u2;
    return mag * cosf(ang);
}

// Helper functions
float ModeSeek::median_of(const std::vector<float> &in) const
{
    if (in.empty())
        return NAN;
    std::vector<float> v = in;
    const size_t n = v.size();
    const size_t mid = n / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    if (n % 2 == 1)
        return v[mid];
    const float a = v[mid];
    const float b = *std::max_element(v.begin(), v.begin() + mid);
    return 0.5f * (a + b);
}

// RSSI simulation methods
float ModeSeek::rssi_ideal_dbm(float d) const
{
    static constexpr float P0_ref_dbm = -40.0f;
    static constexpr float n_pathloss = 2.5f;
    static constexpr float d0_ref_m = 1.0f;

    d = fmaxf(d, d0_ref_m);
    return P0_ref_dbm - 10.0f * n_pathloss * log10f(d / d0_ref_m);
}

float ModeSeek::rssi_noisy_dbm(float d) const
{
    const float ideal = rssi_ideal_dbm(d);
    // Add some realistic noise
    const float noise = 1.75f * SS.rng.normal01();
    return ideal + noise;
}

float ModeSeek::get_telem_rssi_dbm() const
{
#if HAVE_GCS_MAVLINK
    float best_dbm = NAN;
    for (uint8_t i = 0; i < gcs().num_instances(); i++)
    {
        const GCS_MAVLINK *link = gcs().get_link(i);
        if (!link)
            continue;

        // Try different RSSI access methods based on what's available
        int16_t rssi_dbm = INT16_MIN;

        // Method 1: Radio status RSSI (scaled to dBm approximation)
        const mavlink_radio_status_t *radio_status = link->get_radio_status();
        if (radio_status && radio_status->rssi != UINT8_MAX)
        {
            // Convert 0-255 RSSI to approximate dBm (-120 to -30 typical)
            rssi_dbm = -120 + (radio_status->rssi * 90 / 255);
            best_dbm = std::isnan(best_dbm) ? float(rssi_dbm) : std::max(best_dbm, float(rssi_dbm));
        }

        // Method 2: Direct dBm access if available
#ifdef HAVE_GCS_MAVLINK_GET_RADIO_RSSI_DBM
        else if (link->get_radio_rssi_dbm(rssi_dbm) && rssi_dbm != INT16_MIN)
        {
            best_dbm = std::isnan(best_dbm) ? float(rssi_dbm) : std::max(best_dbm, float(rssi_dbm));
        }
#endif
    }
    return best_dbm;
#else
    return NAN;
#endif
}

// One RSSI sample
float ModeSeek::sample_rssi_dbm(float distance_m) const
{
    // Try real telemetry RSSI first
    float real = get_telem_rssi_dbm();

    if (!std::isnan(real))
    {
        // Clamp to reasonable range
        gcs().send_text(MAV_SEVERITY_INFO, "SEEK RSSI: Real RSSI AVAILABLE");
        return clampf(real, -120.0f, -0.0f);
    }

    // Fallback to simulated RSSI
    distance_m = fmaxf(distance_m, 1.0f);
    return rssi_noisy_dbm(distance_m);
}

// Take w quick samples @ fs Hz, median->EMA
bool ModeSeek::sense_rssi_filtered(float distance_m, uint16_t w, float fs_hz, float alpha,
                                   float &ema_last, float &out_filtered)
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t Ts_ms = (uint32_t)(1000.0f / fmaxf(1.0f, fs_hz));

    if (SS.samples_needed == 0)
    {
        SS.samples_needed = (uint16_t)clampf((float)w, 1.0f, 64.0f);
        SS.window.clear();
        SS.window.reserve(SS.samples_needed);
        SS.next_sample_ms = now_ms;
    }

    if (now_ms >= SS.next_sample_ms && SS.samples_needed > 0)
    {
        const float sample = sample_rssi_dbm(distance_m);
        // gcs().send_text(MAV_SEVERITY_INFO, "SEEK RSSI: sample %.1f dBm", (double)sample);
        SS.window.push_back(sample);
        SS.samples_needed--;
        SS.next_sample_ms = now_ms + Ts_ms;
    }

    if (SS.samples_needed > 0)
    {
        return false;
    }

    const float med = median_of(SS.window);
    // gcs().send_text(MAV_SEVERITY_INFO, "SEEK RSSI: median %.1f dBm", (double)med);

    if (std::isnan(ema_last))
    {
        ema_last = med;
    }
    else
    {
        ema_last = (1.0f - alpha) * ema_last + alpha * med;
    }
    out_filtered = ema_last;
    // gcs().send_text(MAV_SEVERITY_INFO, "SEEK RSSI: filtered %.1f dBm", (double)out_filtered);
    // Reset for next sensing
    SS.samples_needed = 0;

    return true;
}

bool ModeSeek::init(bool ignore_checks)
{
    // We NEVER allow skipping checks
    (void)ignore_checks;
    


    if (!params_inited_) {
        AP_Param::setup_object_defaults(this, var_info);  // creates/loads SEEK_* params
        params_inited_ = true;
    }
    load_params_into_runtime(); 

    
    gcs().send_text(MAV_SEVERITY_INFO, "SEEK: initializing");
    Location cur;
    if (!AP::ahrs().get_location(cur))
    {
        gcs().send_text(MAV_SEVERITY_ERROR, "SEEK: no GPS lock");
        return false;
    }

    if (!AP::ahrs().home_is_set())
    {
        gcs().send_text(MAV_SEVERITY_ERROR, "SEEK: home not set - ARM FIRST");
        return false;
    }

    Location home = AP::ahrs().get_home();
    gcs().send_text(MAV_SEVERITY_INFO, "SEEK: home at lat %.7f lon %.7f",
                    (double)home.lat, (double)home.lng);

    last_distance_m_ = cur.get_distance(home);
    gcs().send_text(MAV_SEVERITY_INFO, "SEEK: distance to home %.1fm", (double)last_distance_m_);

    const float MAX_INIT_DIST = 1000.0f; // 1 km
    if (last_distance_m_ > MAX_INIT_DIST)
    {
        gcs().send_text(MAV_SEVERITY_ERROR, "SEEK: too far from home (%.0fm)", (double)last_distance_m_);
        return false;
    }

    have_home_vec_ = update_vector_to_home(cur, home);
    if (!have_home_vec_)
    {
        return false;
    }

    SS = SeekState{};
    SS.window.reserve(64);
    SS.start_distance_m = last_distance_m_;
    SS.initial_step_size_m = fmaxf(SP.min_step_size_m,
                                   fminf(SP.max_step_size_m, SS.start_distance_m / fmaxf(1.0f, SP.step_size_constant)));
    gcs().send_text(MAV_SEVERITY_INFO, "SEEK: init step size %.2fm", (double)SS.initial_step_size_m);
    SS.heading_rad = home_bearing_rad_;
    gcs().send_text(MAV_SEVERITY_INFO, "SEEK: bearing to home %.1fdeg", (double)rad2deg(SS.heading_rad));

    SS.phase = Phase::INIT;
    SS.inited = true;

    // gcs().send_text(MAV_SEVERITY_INFO,
    //                 "SEEK: init d=%.1fm brg=%.1fdeg v=%.2fm/s",
    //                 (double)SS.start_distance_m,
    //                 (double)rad2deg(SS.heading_rad),
    //                 (double)SP.speed_mps);

    return true;
}

bool ModeSeek::update_vector_to_home(Location &cur, Location &home)
{
    if (!AP::ahrs().home_is_set())
        return false;

    home = AP::ahrs().get_home();
    last_distance_m_ = cur.get_distance(home);
    const int32_t bearing_cd = cur.get_bearing_to(home);
    home_bearing_rad_ = radians(((float)bearing_cd) * 0.01f);
    return true;
}

void ModeSeek::run()
{
    Vector3f vel_NEU{0.0f, 0.0f, 0.0f};
    // gcs().send_text(MAV_SEVERITY_INFO, "SEEK: Starting Control Loop");
    Location cur, home;
    if (AP::ahrs().get_location(cur) && AP::ahrs().home_is_set())
    {
        have_home_vec_ = update_vector_to_home(cur, home);
    }
    else
    {
        have_home_vec_ = false;
    }

    // RE-INIT IF NEEDED
    if (!SS.inited)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "SEEK: re-initializing");
        SS = SeekState{};
        SS.window.reserve(64);
        SS.start_distance_m = last_distance_m_;
        SS.initial_step_size_m = fminf(SP.max_step_size_m,
                                       SS.start_distance_m / fmaxf(1.0f, SP.step_size_constant));
        SS.heading_rad = home_bearing_rad_;
        SS.phase = Phase::INIT;
        SS.inited = true;
    }

    // === 4. PHASE LOGIC ===
    switch (SS.phase)
    {
    case Phase::INIT:
    {
        float filt = NAN;
        if (sense_rssi_filtered(last_distance_m_, SP.w, SP.fs, SP.alpha, SS.ema_last, filt))
        {

            SS.rssi_now = SS.rssi_prev = SS.initial_rssi = filt;

            float progress = fabsf(SS.rssi_now - SS.initial_rssi) /
                             fmaxf(1.0f, fabsf(SS.initial_rssi - SP.target_rssi_dbm));
            progress = clampf(progress, 0.0f, 1.0f);

            SS.current_step_len_m = fmaxf(SP.min_step_size_m,
                                          fminf(SP.max_step_size_m,
                                                SS.initial_step_size_m * expf(-SP.decay_rate * progress)));

            float move_time_s = SS.current_step_len_m / fmaxf(0.1f, SP.speed_mps);
            SS.phase_end_ms = AP_HAL::millis() + (uint32_t)(1000.0f * move_time_s);
            SS.phase = Phase::MOVE;

            gcs().send_text(MAV_SEVERITY_INFO,
                            "SEEK:INIT rssi=%.1fdBm step=%.2fm head=%.1f",
                            (double)filt, (double)SS.current_step_len_m, (double)rad2deg(SS.heading_rad));
        }
        break;
    }

    case Phase::MOVE:
    {
        vel_NEU.x = SP.speed_mps * cosf(SS.heading_rad);
        vel_NEU.y = SP.speed_mps * sinf(SS.heading_rad);
        vel_NEU.z = 0.0f;

        // gcs().send_text(MAV_SEVERITY_INFO,
        //                 "SEEK:MOVE v=%.2f,%.2f,%.2f head=%.1f",
        //                 (double)vel_NEU.x, (double)vel_NEU.y, (double)vel_NEU.z,
        //                 (double)rad2deg(SS.heading_rad));


        pos_control->set_vel_desired_NEU_ms(vel_NEU);
        pos_control->update_NE_controller();
        pos_control->update_U_controller();
        if (AP_HAL::millis() >= SS.phase_end_ms)
        {
            SS.phase = Phase::SEEK;
            SS.samples_needed = 0;
            // vel_NEU = {0.0f, 0.0f, 0.0f};
        }

   
        break;
    }

    case Phase::SEEK:
    {
        vel_NEU = Vector3f(0, 0, 0);
        float filtered = NAN;
        if (sense_rssi_filtered(last_distance_m_, SP.w, SP.fs, SP.alpha, SS.ema_last, filtered))
        {
            SS.rssi_now = filtered;
            bool improving = (SS.rssi_now > SS.rssi_prev);
            if (!improving)
            {
                SS.heading_rad += deg2rad(SP.turn_deg);
                while (SS.heading_rad > 2 * M_PI)
                    SS.heading_rad -= 2 * M_PI;
                while (SS.heading_rad < 0)
                    SS.heading_rad += 2 * M_PI;
                gcs().send_text(MAV_SEVERITY_INFO,
                                "SEEK: RSSI WORSE - TURNING NEW HEAD=%.1fdeg",
                                (double)rad2deg(SS.heading_rad));
            }

            
            

            if (SS.rssi_now >= SP.target_rssi_dbm)
            {
                SS.phase = Phase::HOLD;
                gcs().send_text(MAV_SEVERITY_INFO, "SEEK: TARGET RSSI (%.1fdBm)", (double)SS.rssi_now);
                break;
            }

            float progress = fabsf(SS.rssi_now - SS.initial_rssi) /
                             fmaxf(1.0f, fabsf(SS.initial_rssi - SP.target_rssi_dbm));
            progress = clampf(progress, 0.0f, 1.0f);

            SS.current_step_len_m = fmaxf(SP.min_step_size_m,
                                          fminf(SP.max_step_size_m,
                                                SS.initial_step_size_m * expf(-SP.decay_rate * progress)));

            
            float move_time_s = SS.current_step_len_m / fmaxf(0.1f, SP.speed_mps);
            SS.phase_end_ms = AP_HAL::millis() + (uint32_t)(1000.0f * move_time_s);
            SS.step_index++;
            SS.phase = Phase::MOVE;

            gcs().send_text(
                MAV_SEVERITY_INFO,
                "SEEK:SEEK rssi=%.1fdBm head=%.1f progress=%.2f step=%.2f  step_index=%d  move_time=%.1fs improving=%s",
                (double)SS.rssi_now,
                (double)rad2deg(SS.heading_rad),
                (double)progress, 
                (double)SS.current_step_len_m,
                (int)SS.step_index,
                (double)move_time_s,
                improving ? "YES" : "NO"
            );
            SS.rssi_prev = SS.rssi_now;
        }
        break;
    }

    case Phase::HOLD:
    default:
        vel_NEU = Vector3f(0, 0, 0);
        break;
    }
    Vector2f desired_vel_NE = Vector2f(vel_NEU.x, vel_NEU.y);
    Vector2f desired_accel_NE = Vector2f(0.0f, 0.0f);

    // === XY VELOCITY CONTROL ===
    pos_control->input_vel_accel_NE_m(desired_vel_NE, desired_accel_NE);
    float desired_vel_U = 0.0f;
    float desired_accel_U = 0.0f;
    pos_control->input_vel_accel_U_m(desired_vel_U, desired_accel_U);
    pos_control->update_U_controller();

    Vector3f thrust = pos_control->get_thrust_vector();
    attitude_control->input_thrust_vector_heading(
        thrust,
        AC_AttitudeControl::HeadingCommand{SS.heading_rad});

    // === 6. LOGGING ===
    // gcs().send_text(MAV_SEVERITY_DEBUG,
    //                 "SEEK: P=%d V=%.2f,%.2f H=%.1f D=%.1fm",
    //                 (int)SS.phase,
    //                 (double)vel_NEU.x, (double)vel_NEU.y,
    //                 (double)rad2deg(SS.heading_rad),
    //                 (double)last_distance_m_);
}

#endif // MODE_SEEK_ENABLED