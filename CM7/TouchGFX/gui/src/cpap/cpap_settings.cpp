/**
 * cpap_settings.cpp - default values + enum-to-string helpers.
 */
#include <gui/cpap/cpap_settings.hpp>

namespace cpap {

namespace {

Settings g_settings = {
    /* therapy_mode        */ TherapyMode::CPAP,
    /* ramp                */ RampMode::OFF,    /* Off by default; "Auto"  */
                                                /* is AutoRamp (#38).      */
    /* epr                 */ OnOff::OFF,
    /* epr_type            */ EprType::FULL_TIME,
    /* epr_level           */ EprLevel::L1,
    /* smart_start         */ OnOff::OFF,
    /* smart_stop          */ OnOff::OFF,
    /* tube_temp_mode      */ TubeTempMode::AUTO,
    /* tube_temp_c         */ 27U,
    /* climate_mode        */ ClimateMode::AUTO,
    /* humidity            */ HumidityLevel::L4,
    /* airplane_mode       */ OnOff::OFF,
    /* cpap_pressure_half  */ CPAP_PRESSURE_HALF_DEF,    /* 8.0 cmH2O */
    /* apap_min_pressure_half */ APAP_PRESSURE_HALF_MIN_DEF, /* 5.0 cmH2O */
    /* apap_max_pressure_half */ APAP_PRESSURE_HALF_MAX_DEF, /* 20.0 cmH2O */
    /* epr_allowed         */ true,
    /* smart_start_allowed */ true,
    /* smart_stop_allowed  */ true
};

/* Defaults: nothing connected, no humidifier activity. Back-end code
 * (humidifier task, BT stack, modem driver) writes into this freely. */
SystemStatus g_system_status = {
    /* humidifier          */ HumidifierStatus::NORMAL,
    /* bluetooth_connected */ false,
    /* cellular            */ CellularSignal::NONE
};

template <typename E>
E clampEnum(int v)
{
    int max_v = static_cast<int>(E::NUM_VALUES) - 1;
    if (v < 0) v = 0;
    if (v > max_v) v = max_v;
    return static_cast<E>(v);
}

} // namespace

Settings& getSettings()
{
    return g_settings;
}

SystemStatus& getSystemStatus()
{
    return g_system_status;
}

const char* toString(TherapyMode v)
{
    switch (v)
    {
    case TherapyMode::CPAP:             return "CPAP";
    case TherapyMode::AUTOSET:          return "Autoset";
    case TherapyMode::AUTOSET_FOR_HER:  return "Autoset for her";
    default:                            return "?";
    }
}

const char* toString(RampMode v)
{
    switch (v)
    {
    case RampMode::OFF:    return "Off";
    case RampMode::AUTO:   return "Auto";
    case RampMode::MIN_5:  return "5 min";
    case RampMode::MIN_10: return "10 min";
    case RampMode::MIN_15: return "15 min";
    case RampMode::MIN_20: return "20 min";
    case RampMode::MIN_25: return "25 min";
    case RampMode::MIN_30: return "30 min";
    case RampMode::MIN_35: return "35 min";
    case RampMode::MIN_40: return "40 min";
    case RampMode::MIN_45: return "45 min";
    default:               return "?";
    }
}

const char* toString(OnOff v)
{
    return (v == OnOff::ON) ? "On" : "Off";
}

const char* toString(EprType v)
{
    switch (v)
    {
    case EprType::FULL_TIME: return "Full time";
    case EprType::RAMP_ONLY: return "Ramp only";
    default:                 return "?";
    }
}

const char* toString(EprLevel v)
{
    switch (v)
    {
    case EprLevel::L1: return "1";
    case EprLevel::L2: return "2";
    case EprLevel::L3: return "3";
    default:           return "?";
    }
}

const char* toString(TubeTempMode v)
{
    switch (v)
    {
    case TubeTempMode::OFF:    return "Off";
    case TubeTempMode::AUTO:   return "Auto (27 C)";
    case TubeTempMode::MANUAL: return "Manual";
    default:                   return "?";
    }
}

const char* toString(ClimateMode v)
{
    return (v == ClimateMode::AUTO) ? "Auto" : "Manual";
}

const char* toString(HumidityLevel v)
{
    switch (v)
    {
    case HumidityLevel::OFF: return "Off";
    case HumidityLevel::L1:  return "1";
    case HumidityLevel::L2:  return "2";
    case HumidityLevel::L3:  return "3";
    case HumidityLevel::L4:  return "4";
    case HumidityLevel::L5:  return "5";
    case HumidityLevel::L6:  return "6";
    case HumidityLevel::L7:  return "7";
    case HumidityLevel::L8:  return "8";
    default:                 return "?";
    }
}

TherapyMode   step(TherapyMode v,   int d) { return clampEnum<TherapyMode>    (static_cast<int>(v) + d); }
RampMode      step(RampMode v,      int d) { return clampEnum<RampMode>     (static_cast<int>(v) + d); }
OnOff         step(OnOff v,         int d) { return clampEnum<OnOff>        (static_cast<int>(v) + d); }
EprType       step(EprType v,       int d) { return clampEnum<EprType>      (static_cast<int>(v) + d); }
EprLevel      stepEprLevel(EprLevel v, int d)
{
    int x = static_cast<int>(v) + d;
    if (x < static_cast<int>(EprLevel::L1)) x = static_cast<int>(EprLevel::L1);
    if (x > static_cast<int>(EprLevel::L3)) x = static_cast<int>(EprLevel::L3);
    return static_cast<EprLevel>(x);
}
TubeTempMode  step(TubeTempMode v,  int d) { return clampEnum<TubeTempMode> (static_cast<int>(v) + d); }
ClimateMode   step(ClimateMode v,   int d) { return clampEnum<ClimateMode>  (static_cast<int>(v) + d); }
HumidityLevel step(HumidityLevel v, int d) { return clampEnum<HumidityLevel>(static_cast<int>(v) + d); }

uint8_t stepTubeTempC(uint8_t v, int delta)
{
    int x = static_cast<int>(v) + delta;
    if (x < 16) x = 16;
    if (x > 30) x = 30;
    return static_cast<uint8_t>(x);
}

uint8_t stepCpapPressureHalf(uint8_t v, int delta)
{
    int x = static_cast<int>(v) + delta;
    if (x < static_cast<int>(CPAP_PRESSURE_HALF_MIN)) x = CPAP_PRESSURE_HALF_MIN;
    if (x > static_cast<int>(CPAP_PRESSURE_HALF_MAX)) x = CPAP_PRESSURE_HALF_MAX;
    return static_cast<uint8_t>(x);
}

uint8_t stepApapMinPressureHalf(uint8_t min_half, uint8_t max_half, int delta)
{
    int x = static_cast<int>(min_half) + delta;
    const int lo = static_cast<int>(CPAP_PRESSURE_HALF_MIN);
    const int hi = static_cast<int>(max_half) - 1;
    if (x < lo) x = lo;
    if (x > hi) x = hi;
    return static_cast<uint8_t>(x);
}

uint8_t stepApapMaxPressureHalf(uint8_t min_half, uint8_t max_half, int delta)
{
    int x = static_cast<int>(max_half) + delta;
    const int lo = static_cast<int>(min_half) + 1;
    const int hi = static_cast<int>(CPAP_PRESSURE_HALF_MAX);
    if (x < lo) x = lo;
    if (x > hi) x = hi;
    return static_cast<uint8_t>(x);
}

} // namespace cpap
