/**
 * cpap_menu.cpp - Menu state machine and label tables.
 */
#include <gui/cpap/cpap_menu.hpp>
#include <gui/cpap/cpap_settings.hpp>

#include "therapy.h"   /* C therapy state machine; called on START/STOP    */

#include <stdio.h>

namespace cpap {

namespace {

/* ------------------------------------------------------------------------- */
/* List definitions                                                          */
/* ------------------------------------------------------------------------- */

struct ListItem
{
    const char* label;
    Page        target_page;     /* where a click on this item navigates     */
    Mode        target_mode;     /* LIST, VALUE_EDIT, or INFO                */
};

/* HOME page items - top-level. No Back row: this is the root of the
 * navigable hierarchy. To leave HOME the user either starts therapy or
 * enters a sub-list. */
const ListItem k_home_items[] = {
    { "MY OPTIONS",    Page::MY_OPTIONS,    Mode::LIST },
    { "MY SLEEP VIEW", Page::MY_SLEEP_VIEW, Mode::INFO },
    { "MORE",          Page::MORE,          Mode::LIST },
    { "START THERAPY", Page::THERAPY,       Mode::INFO },
};

/* MY OPTIONS submenu. The "<- Back" pseudo-item at index 0 points at HOME,
 * which makes a click on it behave identically to goBack(). itemValue()
 * and the locked-by-care-provider check both treat index 0 as "no value /
 * no lock", so the existing renderListPage logic Just Works. */
const ListItem k_my_options_items[] = {
    { "<- Back",          Page::HOME,                  Mode::LIST },
    { "Mode",             Page::VALUE_THERAPY_MODE,    Mode::VALUE_EDIT },
    { "CPAP Pressure",    Page::VALUE_CPAP_PRESSURE,   Mode::VALUE_EDIT },
    { "Min Pressure",     Page::VALUE_AUTOSET_MIN,     Mode::VALUE_EDIT },
    { "Max Pressure",     Page::VALUE_AUTOSET_MAX,     Mode::VALUE_EDIT },
    { "Ramp Time",        Page::VALUE_RAMP,            Mode::VALUE_EDIT },
    { "Ramp Start",       Page::VALUE_RAMP_START,      Mode::VALUE_EDIT },
    { "Pressure Relief",  Page::VALUE_EPR,             Mode::VALUE_EDIT },
    { "EPR Type",         Page::VALUE_EPR_TYPE,        Mode::VALUE_EDIT },
    { "EPR Level",        Page::VALUE_EPR_LEVEL,       Mode::VALUE_EDIT },
    { "SmartStart",       Page::VALUE_SMART_START,     Mode::VALUE_EDIT },
    { "SmartStop",        Page::VALUE_SMART_STOP,      Mode::VALUE_EDIT },
    { "Tube Temperature", Page::VALUE_TUBE_TEMP_MODE,  Mode::VALUE_EDIT },
    { "Climate Control",  Page::VALUE_CLIMATE,         Mode::VALUE_EDIT },
    { "Humidity Level",   Page::VALUE_HUMIDITY,        Mode::VALUE_EDIT },
};

/* MORE submenu. Back at index 0 returns to HOME. */
const ListItem k_more_items[] = {
    { "<- Back",           Page::HOME,                Mode::LIST },
    { "Mask Fit",          Page::MASK_FIT,            Mode::INFO },
    { "Airplane Mode",     Page::VALUE_AIRPLANE_MODE, Mode::VALUE_EDIT },
    { "myAir / Bluetooth", Page::MYAIR,               Mode::INFO },
    { "Diagnostics",       Page::DIAGNOSTICS,         Mode::INFO },
};

/* True if a list page's row 0 is a "<- Back" pseudo-item. Used by the view
 * to know the initial selection on entry, and by the controller to know
 * the index of the first real item. */
bool listHasBackRow(Page p)
{
    return (p == Page::MY_OPTIONS) || (p == Page::MORE);
}

template <size_t N>
constexpr uint8_t arrLen(const ListItem (&)[N])
{
    return static_cast<uint8_t>(N);
}

const ListItem* itemsForPage(Page p, uint8_t& out_count)
{
    switch (p)
    {
    case Page::HOME:
        out_count = arrLen(k_home_items);
        return k_home_items;
    case Page::MY_OPTIONS:
        out_count = arrLen(k_my_options_items);
        return k_my_options_items;
    case Page::MORE:
        out_count = arrLen(k_more_items);
        return k_more_items;
    default:
        out_count = 0;
        return nullptr;
    }
}

Page parentList(Page p)
{
    switch (p)
    {
    case Page::VALUE_THERAPY_MODE:
    case Page::VALUE_CPAP_PRESSURE:
    case Page::VALUE_AUTOSET_MIN:
    case Page::VALUE_AUTOSET_MAX:
    case Page::VALUE_RAMP:
    case Page::VALUE_RAMP_START:
    case Page::VALUE_EPR:
    case Page::VALUE_EPR_TYPE:
    case Page::VALUE_EPR_LEVEL:
    case Page::VALUE_SMART_START:
    case Page::VALUE_SMART_STOP:
    case Page::VALUE_TUBE_TEMP_MODE:
    case Page::VALUE_TUBE_TEMP_C:
    case Page::VALUE_CLIMATE:
    case Page::VALUE_HUMIDITY:
        return Page::MY_OPTIONS;
    case Page::VALUE_AIRPLANE_MODE:
    case Page::MASK_FIT:
    case Page::MYAIR:
    case Page::DIAGNOSTICS:
        return Page::MORE;
    case Page::MY_SLEEP_VIEW:
    case Page::MY_OPTIONS:
    case Page::MORE:
    case Page::THERAPY:
        return Page::HOME;
    case Page::HOME:
    case Page::WELCOME:
    default:
        return Page::HOME;
    }
}

uint8_t selectedIndexForChild(Page parent, Page child)
{
    uint8_t count;
    const ListItem* items = itemsForPage(parent, count);
    if (items == nullptr) return 0;
    for (uint8_t i = 0; i < count; ++i)
    {
        if (items[i].target_page == child)
        {
            return i;
        }
    }
    return 0;
}

MenuController g_menu;

/* Static buffer used to format dynamic editor value strings (e.g.
 * "27 C" for the tube-temp page). Returned by editorValueText(). */
char g_editor_value_buf[24] = {0};

static void syncEprToTherapy(const Settings& s)
{
    Therapy_SetEprEnabled(s.epr == OnOff::ON);
    if (s.epr == OnOff::ON)
    {
        Therapy_SetEprType((s.epr_type == EprType::RAMP_ONLY) ? 1U : 0U);
        Therapy_SetEprLevel(static_cast<uint8_t>(s.epr_level));
    }
}

static void syncTherapyFromSettings(const Settings& s)
{
    switch (s.therapy_mode)
    {
    case TherapyMode::CPAP:
        Therapy_SetMode(THERAPY_MODE_CPAP);
        Therapy_SetTargetPressure(cpapPressureCmh2o(s.cpap_pressure_half));
        break;
    case TherapyMode::AUTOSET:
        Therapy_SetMode(THERAPY_MODE_AUTOSET);
        Therapy_SetApapMinPressure(cpapPressureCmh2o(s.apap_min_pressure_half));
        Therapy_SetApapMaxPressure(cpapPressureCmh2o(s.apap_max_pressure_half));
        Therapy_SetTargetPressure(cpapPressureCmh2o(s.apap_min_pressure_half));
        break;
    case TherapyMode::AUTOSET_FOR_HER:
        Therapy_SetMode(THERAPY_MODE_AUTOSET_FOR_HER);
        Therapy_SetApapMinPressure(cpapPressureCmh2o(s.apap_min_pressure_half));
        Therapy_SetApapMaxPressure(cpapPressureCmh2o(s.apap_max_pressure_half));
        Therapy_SetTargetPressure(cpapPressureCmh2o(s.apap_min_pressure_half));
        break;
    default:
        break;
    }
    syncEprToTherapy(s);
}

static bool isCpapMode(const Settings& s)
{
    return s.therapy_mode == TherapyMode::CPAP;
}

static bool isAutosetMode(const Settings& s)
{
    return s.therapy_mode != TherapyMode::CPAP;
}

static bool eprSubSettingLocked(const Settings& s)
{
    return !s.epr_allowed || (s.epr != OnOff::ON);
}

} // namespace

MenuController::MenuController()
    : current_page_(Page::WELCOME),
      current_mode_(Mode::INFO),
      selected_index_(0U),
      needs_redraw_(true),
      therapy_running_(false),
      therapy_phase_(0U)
{
}

MenuController& getMenu()
{
    return g_menu;
}

uint8_t MenuController::itemCount() const
{
    uint8_t count = 0U;
    (void)itemsForPage(current_page_, count);
    return count;
}

const char* MenuController::itemLabel(uint8_t idx) const
{
    uint8_t count;
    const ListItem* items = itemsForPage(current_page_, count);
    if (items == nullptr || idx >= count) return nullptr;
    return items[idx].label;
}

const char* MenuController::itemValue(uint8_t idx) const
{
    /* Returns the right-hand current-value string for MY_OPTIONS rows.
     * Other lists don't show values on the right. Index 0 is the Back row
     * and intentionally has no value. */
    if (current_page_ != Page::MY_OPTIONS) return nullptr;

    const Settings& s = getSettings();
    switch (idx)
    {
    case 0: return nullptr;
    case 1: return toString(s.therapy_mode);
    case 2:
        if (isAutosetMode(s)) return "Locked";
        (void)snprintf(g_editor_value_buf, sizeof(g_editor_value_buf),
                       "%.1f cmH2O",
                       static_cast<double>(cpapPressureCmh2o(s.cpap_pressure_half)));
        return g_editor_value_buf;
    case 3:
        if (isCpapMode(s)) return "Locked";
        (void)snprintf(g_editor_value_buf, sizeof(g_editor_value_buf),
                       "%.1f cmH2O",
                       static_cast<double>(cpapPressureCmh2o(s.apap_min_pressure_half)));
        return g_editor_value_buf;
    case 4:
        if (isCpapMode(s)) return "Locked";
        (void)snprintf(g_editor_value_buf, sizeof(g_editor_value_buf),
                       "%.1f cmH2O",
                       static_cast<double>(cpapPressureCmh2o(s.apap_max_pressure_half)));
        return g_editor_value_buf;
    case 5: return toString(s.ramp);
    case 6:
        (void)snprintf(g_editor_value_buf, sizeof(g_editor_value_buf),
                       "%.1f cmH2O",
                       static_cast<double>(cpapPressureCmh2o(s.ramp_start_half)));
        return g_editor_value_buf;
    case 7: return s.epr_allowed         ? toString(s.epr)         : "Locked";
    case 8: return eprSubSettingLocked(s) ? "Locked"               : toString(s.epr_type);
    case 9: return eprSubSettingLocked(s) ? "Locked"               : toString(s.epr_level);
    case 10: return s.smart_start_allowed ? toString(s.smart_start) : "Locked";
    case 11: return s.smart_stop_allowed  ? toString(s.smart_stop)  : "Locked";
    case 12: return toString(s.tube_temp_mode);
    case 13: return toString(s.climate_mode);
    case 14: return toString(s.humidity);
    default: return nullptr;
    }
}

void MenuController::onRotaryDelta(int delta)
{
    if (delta == 0) return;

    if (current_mode_ == Mode::LIST)
    {
        uint8_t count = itemCount();
        if (count == 0U) return;
        int idx = static_cast<int>(selected_index_) + delta;
        /* Clamp - no wrap-around (avoids accidental jumps with bumpy knobs). */
        if (idx < 0) idx = 0;
        if (idx >= static_cast<int>(count)) idx = count - 1;
        if (static_cast<uint8_t>(idx) != selected_index_)
        {
            selected_index_ = static_cast<uint8_t>(idx);
            needs_redraw_ = true;
        }
        return;
    }

    if (current_mode_ == Mode::VALUE_EDIT)
    {
        Settings& s = getSettings();
        switch (current_page_)
        {
        case Page::VALUE_THERAPY_MODE:
            s.therapy_mode = step(s.therapy_mode, delta);
            syncTherapyFromSettings(s);
            break;
        case Page::VALUE_CPAP_PRESSURE:
            if (isCpapMode(s))
            {
                s.cpap_pressure_half = stepCpapPressureHalf(s.cpap_pressure_half, delta);
                Therapy_SetTargetPressure(cpapPressureCmh2o(s.cpap_pressure_half));
            }
            break;
        case Page::VALUE_AUTOSET_MIN:
            if (isAutosetMode(s))
            {
                s.apap_min_pressure_half = stepApapMinPressureHalf(
                    s.apap_min_pressure_half, s.apap_max_pressure_half, delta);
                Therapy_SetApapMinPressure(cpapPressureCmh2o(s.apap_min_pressure_half));
                Therapy_SetTargetPressure(cpapPressureCmh2o(s.apap_min_pressure_half));
            }
            break;
        case Page::VALUE_AUTOSET_MAX:
            if (isAutosetMode(s))
            {
                s.apap_max_pressure_half = stepApapMaxPressureHalf(
                    s.apap_min_pressure_half, s.apap_max_pressure_half, delta);
                Therapy_SetApapMaxPressure(cpapPressureCmh2o(s.apap_max_pressure_half));
            }
            break;
        case Page::VALUE_RAMP:
            s.ramp = step(s.ramp, delta); break;
        case Page::VALUE_RAMP_START:
            /* #5: up to the pressure therapy starts at. */
            s.ramp_start_half = stepRampStartHalf(
                s.ramp_start_half,
                isCpapMode(s) ? s.cpap_pressure_half : s.apap_min_pressure_half,
                delta);
            break;
        case Page::VALUE_EPR:
            if (s.epr_allowed)
            {
                s.epr = step(s.epr, delta);
                syncEprToTherapy(s);
            }
            break;
        case Page::VALUE_EPR_TYPE:
            if (!eprSubSettingLocked(s))
            {
                s.epr_type = step(s.epr_type, delta);
                syncEprToTherapy(s);
            }
            break;
        case Page::VALUE_EPR_LEVEL:
            if (!eprSubSettingLocked(s))
            {
                s.epr_level = stepEprLevel(s.epr_level, delta);
                syncEprToTherapy(s);
            }
            break;
        case Page::VALUE_SMART_START:
            if (s.smart_start_allowed) s.smart_start = step(s.smart_start, delta); break;
        case Page::VALUE_SMART_STOP:
            if (s.smart_stop_allowed)  s.smart_stop  = step(s.smart_stop,  delta); break;
        case Page::VALUE_TUBE_TEMP_MODE:
            s.tube_temp_mode = step(s.tube_temp_mode, delta); break;
        case Page::VALUE_TUBE_TEMP_C:
            s.tube_temp_c = stepTubeTempC(s.tube_temp_c, delta); break;
        case Page::VALUE_CLIMATE:
            s.climate_mode = step(s.climate_mode, delta); break;
        case Page::VALUE_HUMIDITY:
            s.humidity = step(s.humidity, delta); break;
        case Page::VALUE_AIRPLANE_MODE:
            s.airplane_mode = step(s.airplane_mode, delta); break;
        default:
            break;
        }
        needs_redraw_ = true;
        return;
    }

    /* INFO mode: rotation is a no-op except on THERAPY where the wave
     * animation auto-advances regardless. */
}

void MenuController::onButtonPress()
{
    switch (current_mode_)
    {
    case Mode::INFO:
        if (current_page_ == Page::WELCOME)
        {
            goTo(Page::HOME, Mode::LIST, 0);
        }
        else if (current_page_ == Page::THERAPY)
        {
            therapy_running_ = false;
            Therapy_Stop();
            goBack();
        }
        else
        {
            goBack();
        }
        break;

    case Mode::LIST:
        enterSelectedItem();
        break;

    case Mode::VALUE_EDIT:
        /* Commit + back. EPR On chains to type then level editors.
         * TUBE_TEMP_MODE MANUAL chains to the temperature editor. */
        if (current_page_ == Page::VALUE_EPR &&
            getSettings().epr == OnOff::ON &&
            getSettings().epr_allowed)
        {
            goTo(Page::VALUE_EPR_TYPE, Mode::VALUE_EDIT, 0);
        }
        else if (current_page_ == Page::VALUE_EPR_TYPE)
        {
            goTo(Page::VALUE_EPR_LEVEL, Mode::VALUE_EDIT, 0);
        }
        else if (current_page_ == Page::VALUE_TUBE_TEMP_MODE &&
                 getSettings().tube_temp_mode == TubeTempMode::MANUAL)
        {
            goTo(Page::VALUE_TUBE_TEMP_C, Mode::VALUE_EDIT, 0);
        }
        else
        {
            goBack();
        }
        break;
    }
}

void MenuController::onButtonLongPress()
{
    /* Reserved: jump to HOME from anywhere. */
    if (current_page_ != Page::HOME)
    {
        goTo(Page::HOME, Mode::LIST, 0);
    }
}

void MenuController::enterSelectedItem()
{
    uint8_t count;
    const ListItem* items = itemsForPage(current_page_, count);
    if (items == nullptr || selected_index_ >= count) return;

    const ListItem& it = items[selected_index_];

    /* Honour "locked by care provider" - just bounce. */
    const Settings& s = getSettings();
    if (current_page_ == Page::MY_OPTIONS)
    {
        if ((it.target_page == Page::VALUE_CPAP_PRESSURE && isAutosetMode(s)) ||
            ((it.target_page == Page::VALUE_AUTOSET_MIN ||
              it.target_page == Page::VALUE_AUTOSET_MAX) && isCpapMode(s)) ||
            (it.target_page == Page::VALUE_EPR         && !s.epr_allowed) ||
            (it.target_page == Page::VALUE_EPR_TYPE    && eprSubSettingLocked(s)) ||
            (it.target_page == Page::VALUE_EPR_LEVEL   && eprSubSettingLocked(s)) ||
            (it.target_page == Page::VALUE_SMART_START && !s.smart_start_allowed) ||
            (it.target_page == Page::VALUE_SMART_STOP  && !s.smart_stop_allowed))
        {
            needs_redraw_ = true;  /* could flash a "locked" indicator       */
            return;
        }
    }

    if (it.target_page == Page::THERAPY)
    {
        therapy_running_ = true;
        therapy_phase_   = 0U;

        /* Snap the latest editable settings into the therapy back-end,
         * then start. The back-end takes its own snapshot in Start()
         * so post-start setting changes only re-tune live (they don't
         * yank the running ramp). */
        const Settings& s = getSettings();
        syncTherapyFromSettings(s);
        /* Map RampMode enum to minutes; OFF=0, then 5..45 in 5-min steps.
         * AUTO is AutoRamp (#38): hold the start pressure until sleep onset. */
        uint8_t ramp_min = 0U;
        switch (s.ramp)
        {
        case RampMode::OFF:    ramp_min = 0U;  break;
        case RampMode::AUTO:   ramp_min = 0U;  break;
        case RampMode::MIN_5:  ramp_min = 5U;  break;
        case RampMode::MIN_10: ramp_min = 10U; break;
        case RampMode::MIN_15: ramp_min = 15U; break;
        case RampMode::MIN_20: ramp_min = 20U; break;
        case RampMode::MIN_25: ramp_min = 25U; break;
        case RampMode::MIN_30: ramp_min = 30U; break;
        case RampMode::MIN_35: ramp_min = 35U; break;
        case RampMode::MIN_40: ramp_min = 40U; break;
        case RampMode::MIN_45: ramp_min = 45U; break;
        default:               ramp_min = 0U;  break;
        }
        Therapy_SetRampMinutes(ramp_min);
        Therapy_SetRampAuto(s.ramp == RampMode::AUTO);
        Therapy_SetRampStartPressure(cpapPressureCmh2o(s.ramp_start_half));   /* #5 */
        Therapy_Start();
    }

    /* When stepping into a sub-list that carries a Back row at index 0,
     * skip past Back so the cursor lands on the first real action - users
     * shouldn't have to immediately rotate past Back to find Ramp Time. */
    uint8_t initial = 0U;
    if (it.target_mode == Mode::LIST && listHasBackRow(it.target_page))
    {
        initial = 1U;
    }

    goTo(it.target_page, it.target_mode, initial);
}

void MenuController::goBack()
{
    Page parent = parentList(current_page_);
    uint8_t idx = selectedIndexForChild(parent, current_page_);
    goTo(parent, Mode::LIST, idx);
}

void MenuController::goTo(Page p, Mode m, uint8_t initial_index)
{
    current_page_   = p;
    current_mode_   = m;
    selected_index_ = initial_index;
    needs_redraw_   = true;
}

const char* MenuController::editorTitle() const
{
    switch (current_page_)
    {
    case Page::VALUE_THERAPY_MODE:   return "Mode";
    case Page::VALUE_CPAP_PRESSURE:  return "CPAP Pressure";
    case Page::VALUE_AUTOSET_MIN:    return "Min Pressure";
    case Page::VALUE_AUTOSET_MAX:    return "Max Pressure";
    case Page::VALUE_RAMP:           return "Ramp Time";
    case Page::VALUE_RAMP_START:     return "Ramp Start";
    case Page::VALUE_EPR:            return "Pressure Relief (EPR)";
    case Page::VALUE_EPR_TYPE:       return "EPR Type";
    case Page::VALUE_EPR_LEVEL:      return "EPR Level";
    case Page::VALUE_SMART_START:    return "SmartStart";
    case Page::VALUE_SMART_STOP:     return "SmartStop";
    case Page::VALUE_TUBE_TEMP_MODE: return "Tube Temperature";
    case Page::VALUE_TUBE_TEMP_C:    return "Tube Temperature";
    case Page::VALUE_CLIMATE:        return "Climate Control";
    case Page::VALUE_HUMIDITY:       return "Humidity Level";
    case Page::VALUE_AIRPLANE_MODE:  return "Airplane Mode";
    case Page::WELCOME:              return "Welcome";
    case Page::HOME:                 return "Home";
    case Page::MY_OPTIONS:           return "My Options";
    case Page::MY_SLEEP_VIEW:        return "My Sleep View";
    case Page::MORE:                 return "More";
    case Page::MASK_FIT:             return "Mask Fit";
    case Page::MYAIR:                return "myAir";
    case Page::DIAGNOSTICS:          return "Diagnostics";
    case Page::THERAPY:              return "Therapy";
    default:                         return "";
    }
}

const char* MenuController::editorValueText() const
{
    const Settings& s = getSettings();
    switch (current_page_)
    {
    case Page::VALUE_THERAPY_MODE:
        return toString(s.therapy_mode);
    case Page::VALUE_CPAP_PRESSURE:
        (void)snprintf(g_editor_value_buf, sizeof(g_editor_value_buf),
                       "%.1f cmH2O",
                       static_cast<double>(cpapPressureCmh2o(s.cpap_pressure_half)));
        return g_editor_value_buf;
    case Page::VALUE_AUTOSET_MIN:
        (void)snprintf(g_editor_value_buf, sizeof(g_editor_value_buf),
                       "%.1f cmH2O",
                       static_cast<double>(cpapPressureCmh2o(s.apap_min_pressure_half)));
        return g_editor_value_buf;
    case Page::VALUE_AUTOSET_MAX:
        (void)snprintf(g_editor_value_buf, sizeof(g_editor_value_buf),
                       "%.1f cmH2O",
                       static_cast<double>(cpapPressureCmh2o(s.apap_max_pressure_half)));
        return g_editor_value_buf;
    case Page::VALUE_RAMP:           return toString(s.ramp);
    case Page::VALUE_RAMP_START:
        (void)snprintf(g_editor_value_buf, sizeof(g_editor_value_buf),
                       "%.1f cmH2O",
                       static_cast<double>(cpapPressureCmh2o(s.ramp_start_half)));
        return g_editor_value_buf;
    case Page::VALUE_EPR:            return toString(s.epr);
    case Page::VALUE_EPR_TYPE:       return toString(s.epr_type);
    case Page::VALUE_EPR_LEVEL:
        (void)snprintf(g_editor_value_buf, sizeof(g_editor_value_buf),
                       "%s cmH2O", toString(s.epr_level));
        return g_editor_value_buf;
    case Page::VALUE_SMART_START:    return toString(s.smart_start);
    case Page::VALUE_SMART_STOP:     return toString(s.smart_stop);
    case Page::VALUE_TUBE_TEMP_MODE: return toString(s.tube_temp_mode);
    case Page::VALUE_TUBE_TEMP_C:
        (void)snprintf(g_editor_value_buf, sizeof(g_editor_value_buf),
                       "%u C", static_cast<unsigned>(s.tube_temp_c));
        return g_editor_value_buf;
    case Page::VALUE_CLIMATE:        return toString(s.climate_mode);
    case Page::VALUE_HUMIDITY:       return toString(s.humidity);
    case Page::VALUE_AIRPLANE_MODE:  return toString(s.airplane_mode);
    default:                         return "";
    }
}

const char* MenuController::editorHint() const
{
    switch (current_page_)
    {
    case Page::VALUE_THERAPY_MODE:
        return "CPAP / Autoset / Autoset for her   Click: save";
    case Page::VALUE_CPAP_PRESSURE:
        return "Range 4.0 - 25.0 cmH2O   Click: save";
    case Page::VALUE_AUTOSET_MIN:
    case Page::VALUE_AUTOSET_MAX:
        return "Autoset range 4.0 - 25.0 cmH2O   Click: save";
    case Page::VALUE_RAMP:
        return "Rotate: change   Click: save";
    case Page::VALUE_RAMP_START:
        return "4.0 up to the therapy pressure   Click: save";
    case Page::VALUE_TUBE_TEMP_MODE:
        return "Manual? Click for temp.";
    case Page::VALUE_TUBE_TEMP_C:
        return "Range 16 to 30 C";
    case Page::VALUE_HUMIDITY:
        return "0 = off, 1 (low) ... 8 (max)";
    case Page::VALUE_EPR:
        return "On: set type and level";
    case Page::VALUE_EPR_TYPE:
        return "Full time or Ramp only";
    case Page::VALUE_EPR_LEVEL:
        return "Relief 1, 2, or 3 cmH2O";
    case Page::VALUE_SMART_START:
    case Page::VALUE_SMART_STOP:
    case Page::VALUE_CLIMATE:
    case Page::VALUE_AIRPLANE_MODE:
        return "Rotate: toggle   Click: save";
    case Page::WELCOME:
        return "Press the knob to begin";
    case Page::MY_SLEEP_VIEW:
        return "Click to return";
    case Page::MASK_FIT:
        return "Hold mask in place. Click to exit.";
    case Page::MYAIR:
        return "Pairing not active. Click to exit.";
    case Page::DIAGNOSTICS:
        return "Live updates. Click to return.";
    case Page::THERAPY:
        return "Click to stop therapy";
    default:
        return "";
    }
}

} // namespace cpap
